import mujoco as mj
import numpy as np
import matplotlib.pyplot as plt

class Agent:

    def __init__(self, xml_path):

        # Generate model and data from xml file
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.dt = 0.01
        self.integral_0 = 0
        self.integral_1 = 0
        self.windupmax = 0.2

        # assistive velocity based on the load
        self.v0 = 0.5
        self.v1 = 1.0
        self.previous = -1  # Ensure initial change detection

        # Joint 0
        self.q_init_0 = self.data.qpos[0].copy()
        self.q_end_0 = self.q_init_0
        # Joint 1
        self.q_init_1 = self.data.qpos[1].copy()
        self.q_end_1 = 2.30  # np.pi/2

        # Store parameters for the control
        self.a_1 = np.zeros(4)
        self.previous = -1 #None

        # Controller gains
        self.kp = 100
        self.kd = 100
        self.ki = 100

        # Mass matrix
        self.M = np.zeros((2, 2))

        # To print the graphs
        self.t = []
        self.qact0 = []
        self.qref0 = []
        self.qact1 = []
        self.qref1 = []

    def step_simulation(self):
        mj.mj_step(self.model, self.data)

    def graph_plot(self, filename, filename2):
        plt.figure(1)

        plt.subplot(2, 1, 1)
        plt.plot(self.t, self.qact0, 'k', label='Actual Trajectory 0')
        plt.plot(self.t, self.qref0, 'r--', label='Reference Trajectory 0')
        plt.ylabel('Joint Trajectory 0')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.t, self.qact1, 'k', label='Actual Trajectory 1')
        plt.plot(self.t, self.qref1, 'r--', label='Reference Trajectory 1')
        plt.ylabel('Joint Trajectory 1')
        plt.legend()

        plt.tight_layout()
        plt.savefig(filename, format='png')
        print(f"Plot saved as {filename}")

        plt.figure(2)

        plt.subplot(2, 1, 1)
        plt.plot(self.t, np.subtract(self.qref0, self.qact0), 'k')
        plt.ylabel('Error Position Joint 0')

        plt.subplot(2, 1, 2)
        plt.plot(self.t, np.subtract(self.qref1, self.qact1), 'k')
        plt.ylabel('Error Position Joint 1')

        plt.tight_layout()
        plt.savefig(filename2, format='png')
        print(f"Plot saved as {filename2}")
    
    def generate_trajectory(self, t0, tf, q0, qf):
        if tf == t0:
            return q0, 0, 0, 0  # No motion needed if start and end times are the same

        delta_t = tf - t0
        tf_t0_3 = delta_t ** 3

        a0 = qf * (t0**2) * (3*tf - t0) + q0 * (tf**2) * (tf - 3*t0)
        a0 /= tf_t0_3

        a1 = 6 * t0 * tf * (q0 - qf)
        a1 /= tf_t0_3

        a2 = 3 * (t0 + tf) * (qf - q0)
        a2 /= tf_t0_3

        a3 = 2 * (q0 - qf)
        a3 /= tf_t0_3

        return a0, a1, a2, a3

    def init_controller(self):

        #end simulation
        self.end_sim = False

        self.t_end = 0
        self.t_init = 0

        #reference trajectory joint 1, paramenters of cubic polynomial
        self.a_1 = self.generate_trajectory(self.t_init, self.t_end, self.q_init_1, self.q_end_1)

        #reference trajectory of joint 0
        self.q_ref0 = self.q_init_0
        self.qdot_ref0 = 0

    def controller(self, classification_result):

        time = self.data.time #current time 

        self.q_init_1 = self.data.qpos[1] #current position of robot

        #check the classification result, if not 1 or 0 then do nothing 
        if self.previous != classification_result.value:

            #remove the time of classification to make the robot more reactive
            self.t_init = time - 0.3 # 300 ms due to the classification, "anticipated the movement when creating the trajectory"

            if classification_result.value == 0 : #NO LOAD
                self.t_end = 6
            
            elif classification_result.value == 1: #LOAD
                self.t_end = 4

            self.a_1 = self.generate_trajectory(self.t_init, self.t_end, self.q_init_1, self.q_end_1)               
 
            self.previous = classification_result.value #shared value between processes

            
        elapsed_time = time #- self.t_init
        
        if elapsed_time > self.t_end:
            elapsed_time = self.t_end

        if elapsed_time < self.t_init:
            elapsed_time = self.t_init

        q_ref1 = self.a_1[0] + self.a_1[1] * elapsed_time + self.a_1[2] * (elapsed_time ** 2) + self.a_1[3] * (elapsed_time ** 3)
        qdot_ref1 = self.a_1[1] + 2 * self.a_1[2] * elapsed_time + 3 * self.a_1[3] * (elapsed_time ** 2)


        #PID CONTROLLER + FEEDBACK LINEARIZATION

        self.integral_0 += -(self.ki) * (self.data.qpos[0] - self.q_ref0) * self.dt
        self.integral_1 += -self.ki * (self.data.qpos[1] - q_ref1) * self.dt

        if self.windupmax != 0: #anti  windup control
            self.integral_0 = np.clip(self.integral_0, -self.windupmax, self.windupmax)
            self.integral_1 = np.clip(self.integral_1, -self.windupmax, self.windupmax)

        pid_0 = -(self.kp) * (self.data.qpos[0] - self.q_ref0) - (self.kd) * (self.data.qvel[0] - self.qdot_ref0) + self.integral_0
        pid_1 = -self.kp * (self.data.qpos[1] - q_ref1) - self.kd * (self.data.qvel[1] - qdot_ref1) + self.integral_1

        pid_control = np.array([pid_0, pid_1])

        #robot dynamics
        mj.mj_fullM(self.model, self.M, self.data.qM)
        f0 = self.data.qfrc_bias[0]
        f1 = self.data.qfrc_bias[1]
        f = np.array([f0, f1])

        tau_M_pd_control = np.matmul(self.M, pid_control)
        tau = np.add(tau_M_pd_control, f)
        self.data.ctrl[0] = tau[0]
        self.data.ctrl[1] = tau[1]

        #to plot graphs
        self.t.append(self.data.time)
        self.qact0.append(self.data.qpos[0])
        self.qact1.append(self.data.qpos[1])
        self.qref0.append(self.q_ref0)
        self.qref1.append(q_ref1)

        if abs(self.data.qpos[1] - self.q_end_1) < 0.005: #threshold of the error
            self.end_sim = True
        
        return self.end_sim