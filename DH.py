"""

Code for the Denavit-Hartenberg (DH) parameters and forward kinematics. 
Developed by: João Vítor Franke Goetz. 2025.

Parameters Input example:
    Dh_param = [
        {'type': 'revolute', 'a':0, 'alpha': 0, 'd': 0,
         'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}},

        {'type': 'prismatic', 'a':150, 'alpha': 0, theta: 0,
         'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}}, 
    ]

#To plot the robot and evaluate it, we need to provide the values of the variables. including other parameters that are not theta
    variable_values = {
    #Variables not provided in a first moment (Not defined in the Dh_param):
    Mechanism.a[0]: 150,           #a_0 for the first cylindrical joint 150mm
    Mechanism.alpha[1]: 0.01,      #alpha_0 for the first cylindrical joint 0.01 radians
    #...#
    #Variables of movement:
    Mechanism.theta[0]: sp.pi/12,   #theta_0 for the first cylindrical joint 15 degrees
    Mechanism.d[1]: 15,             #d_1 for the second Prismatic joint 45 degrees
    #...#
    #Variables of errors:
    Mechanism.epsilon[0]: 0.0,       #epsilon_0 for the first cylindrical joint
    Mechanism.beta[1]: 0.0,       #epsilon_1 for the second Prismatic joint
    #...#
    } 

    Fuctions used:
    robot = Mechanism(Dh_param)
    #Create the robot with the DH parameters

    robot.plot_mechanism(variable_values, title ='Your Title',initial_config=True)
    #Plot the robot with the initial configuration

    '''Begin the calculations algebrically'''
    robot.forward_kinematics(False)
    #Return the forward kinematics Matrix without errors

    robot.forward_kinematics(True)
    #Return the forward kinematics Matrix with errors apllied

    '''Begin the calculations numerically'''
    robot.evaluate_param(Matrix, variable_values)
    #Return the numerical transformation matrix and position without errors

    robot.evaluate_param(Matrix, variable_values, apply_errors=True)
    #Return the numerical transformation matrix and position with errors

    robot.evaluate_error(position_no_error, position_with_error)
    #Return the error between two positions

     #CERTIFY ALL THE VARIABLES ARE PROVIDED BEFORE PLOTTING#
     
    robot.plot_mechanism(variable_values, title= 'Your Title', initial_config=False)
    #Plot the robot with the initial configuration and the final configuration with and without errors

"""

import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

class Mechanism:
    def __init__(self, param):
        """Initialize the mechanism with the DH parameters."""

        self.param = param
        self.n_joints = len(param)
        #print("Joint Numbers:", self.n_joints) #Debugging

        # Defining symbols for DH:
        self.theta = []
        self.d = []
        self.a = [sp.Symbol(f'a_{i}') for i in range(self.n_joints)]
        self.alpha = [sp.Symbol(f'alpha_{i}') for i in range(self.n_joints)]
        self.phi = [sp.Symbol(f'phi_{i}') for i in range(self.n_joints)]
        self.epsilon = [sp.Symbol(f'epsilon_{i}') for i in range(self.n_joints)]
        self.sigma = [sp.Symbol(f'sigma_{i}') for i in range(self.n_joints)]
        self.beta = [sp.Symbol(f'beta_{i}') for i in range(self.n_joints)]

        for i, params in enumerate(param):
            if params['type'] in ['revolute', 'cylindrical']:
                self.theta.append(sp.Symbol(f'theta_{i}'))
            elif params['type'] == 'prismatic':
                self.theta.append(sp.Symbol(f'theta_{i}'))
            self.d.append(sp.Symbol(f'd_{i}'))

    def dh_matrix(self,i,apply_errors=False):
      """Return the symbolic homogeneous matrix. Internal Use"""

      #Nominal or error values
      a = self.a[i] + (self.sigma[i] if apply_errors else 0)
      alpha = self.alpha[i] + (self.beta[i] if apply_errors else 0)
      d = self.d[i] + (self.epsilon[i] if apply_errors else 0)
      theta = self.theta[i] + (self.phi[i] if apply_errors else 0)
      
      cos_theta = sp.cos(theta)
      sin_theta = sp.sin(theta)
      cos_alpha = sp.cos(alpha)
      sin_alpha = sp.sin(alpha)
      
      return sp.Matrix([
          [cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, a * cos_theta],
          [sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
          [         0,             sin_alpha,              cos_alpha,             d],
          [         0,                      0,                      0,            1]
      ])
    
    def forward_kinematics(self, apply_errors=False):
        """Return the symbolic forward kinematics matrix and position."""

        T = sp.eye(4)  #Matrix 4x4 identity
        for i in range(self.n_joints):
            Ti = self.dh_matrix(i, apply_errors)
            T = T * Ti
            """
            print(f"\nMatrix {i}:")
            A, pos = self.evaluate_param(T)
            for m in range(A.shape[0]):
                for n in range(A.shape[1]):
                    print(f"\n Element ({m},{n}):", A[m,n], end=" ")
            print("\n")
            """
        position = T[:3, 3]
        return T, position
    
    def evaluate_param(self, T, variable_values=None, apply_errors=False):
        """Return the numerical transformation matrix and position."""

        #Build a dictionary with the parameters
        subs_dict = {}
        for i, params in enumerate(self.param):
            #Only substitute parameters if they exist in params
            if 'a' in params:
                subs_dict[self.a[i]] = params['a']
            if 'alpha' in params:
                subs_dict[self.alpha[i]] = params['alpha']
            if 'd' in params:
                subs_dict[self.d[i]] = params['d']
            if 'theta' in params:
                subs_dict[self.theta[i]] = params['theta']
            
            #Handle theta based on joint type
            if params.get('type') == 'prismatic':
                if 'theta_offset' in params:
                    subs_dict[self.theta[i]] = params['theta_offset']
            else:  # revolute or cylindrical
                if 'theta' in params:
                    subs_dict[self.theta[i]] = params['theta']
            
            #Add errors if apply_errors is True, only if they exist
            if apply_errors and 'errors' in params:
                errors = params['errors']
                if 'phi' in errors:
                    subs_dict[self.phi[i]] = errors['phi']
                
                if 'epsilon' in errors:
                    subs_dict[self.epsilon[i]] = errors['epsilon']
                
                if 'sigma' in errors:
                    subs_dict[self.sigma[i]] = errors['sigma']
                
                if 'beta' in errors:
                    subs_dict[self.beta[i]] = errors['beta']
                
            else:
                #If no errors are applied, set error terms to 0 only if not already in subs_dict
                if self.phi[i] not in subs_dict:
                    subs_dict[self.phi[i]] = 0
                if self.epsilon[i] not in subs_dict:
                    subs_dict[self.epsilon[i]] = 0
                if self.sigma[i] not in subs_dict:
                    subs_dict[self.sigma[i]] = 0
                if self.beta[i] not in subs_dict:
                    subs_dict[self.beta[i]] = 0
    
        #Only apply variable_values if provided; otherwise, keep symbolic
        if variable_values:
            valid_symbols = (
                self.a + self.alpha + self.d + self.theta +
                self.phi + self.epsilon + self.sigma + self.beta
            )
            for symbol, value in variable_values.items():
                if symbol in valid_symbols: subs_dict[symbol] = value
        
        #Substitute values into the transformation matrix
        T_numerica = T.subs(subs_dict)
        return T_numerica, T_numerica[:3, 3]
    
    def get_joint_positions(self, variable_values, apply_errors=False):
        """Return the joint positions for the mechanism in 3D. Internal Use"""
        
        positions = [[0, 0, 0]] #Origin
        T = sp.eye(4)
        for i in range(self.n_joints):
            T = T * self.dh_matrix(i, apply_errors)
            T_eval, pos_eval = self.evaluate_param(T, variable_values, apply_errors)
            positions.append(pos_eval)
        
        #Try to convert all positions to float, raising an error if any symbol is left
        try:
            positions_num = np.array([[float(coord) for coord in pos] for pos in positions])
            return positions_num
        except TypeError:
            print("Aviso: Posições contêm símbolos não substituídos:", positions)
            raise TypeError("Todos os símbolos devem ser substituídos por valores numéricos para plotagem")
    
    def evaluate_error(self, pos_no_error, pos_with_error):
        """Return the error between two positions in 3D."""
        try:
            position_no_error = np.array([float(coord) for coord in pos_no_error])
            position_with_error = np.array([float(coord) for coord in pos_with_error])
        except (TypeError, ValueError) as e:
            raise ValueError("As posições devem ser vetores 3D com valores numéricos") from e

        #Verify if the vectors have the correct size
        if len(position_no_error) != 3 or len(position_with_error) != 3:
            raise ValueError("As posições devem ser vetores 3D (x, y, z)")

        #Evaluate real error position
        error = np.linalg.norm(position_no_error - position_with_error)
        return error

    def plot_mechanism(self, variable_values=None, title=None, initial_config=False, plot_type='3d'):
        """Plot the mechanism in 2D or 3D based on user choice with optional variable values."""
        # Validate plot_type
        if plot_type not in ['2d', '3d']:
            raise ValueError("plot_type must be '2d' or '3d'")

        # Create figure
        if plot_type == '3d':
            fig = plt.figure(figsize=(12, 5))
            ax = fig.add_subplot(111, projection='3d')
        else:  # plot_type == '2d'
            fig, ax = plt.subplots(figsize=(10, 10))

        # Define initial configuration values (theta = 0, d = 0, a = 0 where applicable)
        initial_values = {}
        for i, params in enumerate(self.param):
            if 'theta' not in params:
                initial_values[self.theta[i]] = 0           
            if 'a' not in params:
                initial_values[self.a[i]] = 0
            if 'alpha' not in params:
                initial_values[self.alpha[i]] = 0
            if 'd' not in params:
                initial_values[self.d[i]] = 0
            # Ensure errors are zero for initial config
            initial_values[self.phi[i]] = 0
            initial_values[self.epsilon[i]] = 0
            initial_values[self.sigma[i]] = 0
            initial_values[self.beta[i]] = 0
        
        # Calculate initial configuration
        try:
            positions_initial = self.get_joint_positions(initial_values, apply_errors=False)
        except TypeError as e:
            raise TypeError("Could not compute initial configuration") from e

        if initial_config:
            # Plot only initial configuration, optionally with variable_values
            if variable_values:
                try:
                    positions_initial = self.get_joint_positions(variable_values, apply_errors=False)
                except TypeError as e:
                    raise TypeError("Could not apply variable values to initial configuration") from e
            
            if plot_type == '3d':
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], positions_initial[:, 2], 
                        'm-o', label='Initial Configuration')
            else:  # 2d
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], 'm-o', label='Initial Configuration')
        else:
            # Calculate full kinematics with provided variable_values
            try:
                joints_no_error = self.get_joint_positions(variable_values, apply_errors=False)
                joints_with_error = self.get_joint_positions(variable_values, apply_errors=True)
                pos_no_error = joints_no_error[-1]
                pos_with_error = joints_with_error[-1]
                error_vector = pos_with_error - pos_no_error
            except TypeError as e:
                raise TypeError("Could not convert values") from e
            
            if plot_type == '3d':
                # 3D Plot
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], positions_initial[:, 2], 
                        'm-o', label='Initial Configuration')
                ax.plot(joints_no_error[:, 0], joints_no_error[:, 1], joints_no_error[:, 2], 
                        'b-o', label='Without errors')
                ax.plot(joints_with_error[:, 0], joints_with_error[:, 1], joints_with_error[:, 2], 
                        'r--o', label='With errors')
                ax.quiver(
                    pos_no_error[0], pos_no_error[1], pos_no_error[2],
                    error_vector[0], error_vector[1], error_vector[2],
                    color='g', linewidth=2, label='Error vector'
                )
            else:  # 2d
                # 2D Plot (XY plane)
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], 'm-o', label='Initial Configuration')
                ax.plot(joints_no_error[:, 0], joints_no_error[:, 1], 'b-o', label='Without errors')
                ax.plot(joints_with_error[:, 0], joints_with_error[:, 1], 'r--o', label='With errors')
                ax.quiver(
                    pos_no_error[0], pos_no_error[1],
                    error_vector[0], error_vector[1],
                    color='g', label='Error vector'
                )

        # Common plot settings
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        if plot_type == '3d':
            ax.set_zlabel('Z (mm)')
            ax.set_title(title if title else 'Mechanism Plot (3D)')
            ax.view_init(elev=20, azim=45)
        else:
            ax.set_title(title if title else 'Mechanism Plot (2D - XY Plane)')
            ax.axis('equal')  # Maintain aspect ratio in 2D
            ax.grid(True)
        
        ax.legend()
        plt.show()