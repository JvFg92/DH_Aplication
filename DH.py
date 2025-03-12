"""
Code for the Denavit-Hartenberg (DH) parameters and forward kinematics. Developed by: João Vítor Franke Goetz.
"""

import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

"""
Parameters example:
Dh_param = [
        {'type': 'revolute', 'a':0, 'alpha': 0, 'd': 0,
         'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}},

        {'type': 'prismatic', 'a':150, 'alpha': 0, theta_offset: 0,
         'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0,}}, 
    ]

#To plot the robot, we need to provide the values of the variables. including other parameters that are not theta
    variable_values = {
    #Variables not provided in a first moment (Not defined in the Dh_param):
    Mechanism.a[0]: 150,           #a_0 for the first cylindrical joint 150mm
    Mechanism.alpha[1]: 0.01,      #alpha_0 for the first cylindrical joint 0.01 radians

    #Variables of movement:
    Mechanism.theta[0]: sp.pi/12,   #theta_0 for the first cylindrical joint 15 degrees
    Mechanism.d[1]: 15,             #d_1 for the second Prismatic joint 45 degrees
    
    } 

"""

class Mechanism:
    def __init__(self, param):
        self.param = param
        self.n_joints = len(param)
        #print("Joint Numbers:", self.n_joints) #Debugging

        #Defining symbols for DH:
        self.theta = []
        self.d = []
        self.a = [sp.Symbol(f'a_{i}') for i in range(self.n_joints)]
        self.alpha = [sp.Symbol(f'alpha_{i}') for i in range(self.n_joints)]
        self.phi = []
        self.epsilon = []
        self.sigma = [sp.Symbol(f'sigma_{i}') for i in range(self.n_joints)]
        self.beta = [sp.Symbol(f'beta_{i}') for i in range(self.n_joints)]

        for i, params in enumerate(param):
            if params['type'] in ['revolute', 'cylindrical']:
                self.theta.append(sp.Symbol(f'theta_{i}'))
                self.phi.append(sp.Symbol(f'phi_{i}'))
            elif params['type'] == 'prismatic':
                self.theta.append(0)  #theta normally is fixed (0) for prismatic joints
                self.phi.append(sp.Symbol(f'phi_{i}'))
            self.d.append(sp.Symbol(f'd_{i}'))
            self.epsilon.append(sp.Symbol(f'epsilon_{i}'))

    def dh_matrix(self,i,apply_errors=False):
      """Return the symbolic homogeneous matrix."""
      params = self.param[i]
      #Nominal or error values
      a = self.a[i] + (self.sigma[i] if apply_errors else 0)
      alpha = self.alpha[i] + (self.beta[i] if apply_errors else 0)
      d = self.d[i] + (self.epsilon[i] if apply_errors else 0)
      
      if params['type'] in ['revolute', 'cylindrical']:
          theta = self.theta[i] + (self.phi[i] if apply_errors else 0)
      else:  #prismatic
          theta = params.get('theta_offset', 0) + (self.phi[i] if apply_errors else 0)

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
        T = sp.eye(4)  #Matrix 4x4 identity
        for i in range(self.n_joints):
            Ti = self.dh_matrix(i, apply_errors)
            """
            #Validating the matrix:
            print(f"\nMatrix {i}:\n", Tiself.evaluate_param(Ti))
            """
            if(i==0):T=Ti
            else:
                T = T * Ti
                """
                #Validating the matrix:
                print(f"Matrix {i}:")
                A = self.evaluate_param(T)
                for m in range(A.shape[0]):
                    for n in range(A.shape[1]):
                        print(f"\n Element ({m},{n}):", A[m,n], end=" ")
                print("\n")
                """
        position = T[:3, 3]
        return T, position
    
    def evaluate_param(self, T, variable_values=None, apply_errors=False):
        """Evaluate the position of the effector using params from self.param and optional variable values."""
        
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
                else: subs_dict[self.phi[i]] = 0
                if 'epsilon' in errors:
                    subs_dict[self.epsilon[i]] = errors['epsilon']
                else: subs_dict[self.epsilon[i]] = 0
                if 'sigma' in errors:
                    subs_dict[self.sigma[i]] = errors['sigma']
                else: subs_dict[self.sigma[i]] = 0
                if 'beta' in errors:
                    subs_dict[self.beta[i]] = errors['beta']
                else: subs_dict[self.beta[i]] = 0
            else:
                
                #If no errors are applied, explicitly set error terms to 0 only if not already in subs_dict
                if self.phi[i] not in subs_dict:
                    subs_dict[self.phi[i]] = 0
                if self.epsilon[i] not in subs_dict:
                    subs_dict[self.epsilon[i]] = 0
                if self.sigma[i] not in subs_dict:
                    subs_dict[self.sigma[i]] = 0
                if self.beta[i] not in subs_dict:
                    subs_dict[self.beta[i]] = 0
        
        #Add variable values if provided
        if variable_values:
            subs_dict.update(variable_values)

        #Substitute only the provided values, leaving missing ones as symbols
        T_numerica = T.subs(subs_dict)
        return T_numerica, T_numerica[:3, 3]
    
    def get_joint_positions(self, variable_values, apply_errors=False):
        """Return the joint positions for the mechanism in 3D."""
        
        positions = [[0, 0, 0]] #Origin
        T = sp.eye(4)
        for i in range(self.n_joints):
            T = T * self.dh_matrix(i, apply_errors)
            T_eval, pos_eval = self.evaluate_param(T, variable_values, apply_errors)
            pos_num = [float(pos_eval[0]), float(pos_eval[1]), float(pos_eval[2])]
            positions.append(pos_num)
        return np.array(positions)
    
    def evaluate_error(self, pos_no_error, pos_with_error):
        try:
            position_no_error = np.array([float(coord) for coord in pos_no_error])
            position_with_error = np.array([float(coord) for coord in pos_with_error])
        except (TypeError, ValueError) as e:
            raise ValueError("As posições devem ser vetores 3D com valores numéricos") from e

        # Verificar se os vetores têm dimensão 3
        if len(position_no_error) != 3 or len(position_with_error) != 3:
            raise ValueError("As posições devem ser vetores 3D (x, y, z)")

        # Calcular o erro posicional real
        error = np.linalg.norm(position_no_error - position_with_error)
        return error

    def plot_mechanism(self, variable_values=None, title=None, initial_config=False):
        """
        Plot the mechanism in 3D. Can show either the initial configuration or the full
        forward kinematics with error vector.

        Args:
            variable_values: Dictionary with variable values (optional).
            title: Title of the plot (optional).
            initial_config: If True, plots the initial configuration without full kinematics.
                            If False, plots the full kinematics with error vector (default).
        """
        if initial_config:
            # Plot initial configuration without full forward kinematics
            positions = [[0, 0, 0]]  # Origin
            
            # Build initial substitution dictionary
            subs_dict = {}
            for i, params in enumerate(self.param):
                if 'a' in params:
                    subs_dict[self.a[i]] = params['a']
                if 'alpha' in params:
                    subs_dict[self.alpha[i]] = params['alpha']
                if 'd' in params:
                    subs_dict[self.d[i]] = params['d']
                if params['type'] in ['revolute', 'cylindrical']:
                    subs_dict[self.theta[i]] = 0  # Default theta = 0
                elif params['type'] == 'prismatic':
                    subs_dict[self.theta[i]] = params.get('theta_offset', 0)
                    subs_dict[self.d[i]] = 0  # Default d = 0
                subs_dict[self.phi[i]] = 0
                subs_dict[self.epsilon[i]] = 0
                subs_dict[self.sigma[i]] = 0
                subs_dict[self.beta[i]] = 0
            
            # Update with provided variable values
            if variable_values:
                subs_dict.update(variable_values)
            
            # Calculate joint positions for initial config
            T = sp.eye(4)
            for i in range(self.n_joints):
                T_i = self.dh_matrix(i, apply_errors=False)
                if i == 0:
                    T = T_i
                else:
                    T = T * T_i
                pos = T[:3, 3].subs(subs_dict)
                try:
                    pos_num = [float(pos[0]), float(pos[1]), float(pos[2])]
                except (TypeError, ValueError):
                    pos_num = [0, 0, 0]  # Fallback if symbolic
                positions.append(pos_num)
            
            positions = np.array(positions)
            
            # Create figure for initial config
            fig = plt.figure(figsize=(12, 5))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'm-o', label='Initial Configuration no error')
        
        else:
            # Plot full kinematics with error vector
            try:
                joints_no_error = self.get_joint_positions(variable_values, apply_errors=False)
                joints_with_error = self.get_joint_positions(variable_values, apply_errors=True)
                pos_no_error = joints_no_error[-1]
                pos_with_error = joints_with_error[-1]
                error_vector = pos_with_error - pos_no_error
            except TypeError as e:
                raise TypeError("Could not convert values") from e
            
            # Create figure for full kinematics
            fig = plt.figure(figsize=(12, 5))
            ax = fig.add_subplot(111, projection='3d')
            ax.plot(joints_no_error[:, 0], joints_no_error[:, 1], joints_no_error[:, 2], 'b-o', label='Without errors')
            ax.plot(joints_with_error[:, 0], joints_with_error[:, 1], joints_with_error[:, 2], 'r--o', label='With errors')
            ax.quiver(
                pos_no_error[0], pos_no_error[1], pos_no_error[2],
                error_vector[0], error_vector[1], error_vector[2],
                color='g', linewidth=2, label='Error vector'
            )
        
        # Common plot settings
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(title if title else 'Mechanism Plot')
        ax.legend()
        ax.view_init(elev=20, azim=45)
        plt.show()