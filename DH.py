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
    #Return the numerical transformation matrix, position and orientation without errors

    robot.evaluate_param(Matrix, variable_values, apply_errors=True)
    #Return the numerical transformation matrix, position and orientation with errors

    robot.evaluate_error(position_no_error, position_with_error)
    #Return the error between two positions

    robot.get_euler_angles(apply_error)
    #Convert a rotation matrix to Euler angles

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
        
        Matrix = sp.eye(4)  
        Matrix_e = sp.eye(4)

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
        T=sp.simplify(T)
        position = T[:3, 3]
        if(apply_errors): 
            self.Matrix_al_e = T
        else: 
            self.Matrix_al = T
        return T, position
    
    def evaluate_param(self, T=None, variable_values=None, apply_errors=False):
        """Return the numerical transformation matrix and position."""
        #Check if T is a valid matrix
        if (T is None) or (not isinstance(T, sp.Matrix)):
            T,pos=self.forward_kinematics(apply_errors)
           
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
        if(variable_values is not None):
            T_numerica_eval = np.array(T_numerica, dtype=float)
            position_eval = T_numerica_eval[:3, 3]
            orientation_eval = T_numerica_eval[:3, :3]

            if(apply_errors==True):
                self.Matrix_e = T_numerica_eval
            else: 
                self.Matrix = T_numerica_eval
            
            return T_numerica_eval, position_eval, orientation_eval
        else: #Return symbolic matrix, position and orientation
            return T_numerica, T_numerica[:3, 3], T_numerica[:3, :3]  
    
    def get_joint_positions(self, variable_values, apply_errors=False):
        """Return the joint positions for the mechanism in 3D. Internal Use"""
        
        positions = [[0, 0, 0]] #Origin
        T = sp.eye(4)
        for i in range(self.n_joints):
            T = T * self.dh_matrix(i, apply_errors)
            T_eval, pos_eval, orientation = self.evaluate_param(T, variable_values, apply_errors)
            positions.append(pos_eval)
        
        #Try to convert all positions to float, raising an error if any symbol is left
        try:
            positions_num = np.array([[float(coord) for coord in pos] for pos in positions])
            return positions_num
        except TypeError:
            print("Aviso: Posições contêm símbolos não substituídos:", positions)
            raise TypeError("Todos os símbolos devem ser substituídos por valores numéricos para plotagem")
    
    def evaluate_error(self, pos_no_error=None, pos_with_error=None):
        """Return the error between two 3D positions."""
        
        #Ensure matrices are initialized
        if pos_no_error is None:
            if not hasattr(self, 'Matrix') or self.Matrix is None:
                raise ValueError("Nominal matrix not computed. Call evaluate_param with apply_errors=False first.")
            pos_no_error = self.Matrix[:3, 3]
        if pos_with_error is None:
            if not hasattr(self, 'Matrix_e') or self.Matrix_e is None:
                raise ValueError("Error matrix not computed. Call evaluate_param with apply_errors=True first.")
            pos_with_error = self.Matrix_e[:3, 3]
        
        #Validate input types and convert to numpy arrays
        try:
            pos_no_error = np.array(pos_no_error, dtype=float).reshape(3)
            pos_with_error = np.array(pos_with_error, dtype=float).reshape(3)
        except (TypeError, ValueError) as e:
            raise ValueError("Positions must be 3D vectors with numerical values. Ensure all symbols are substituted.") from e
        
        #Verify 3D vectors
        if pos_no_error.shape != (3,) or pos_with_error.shape != (3,):
            raise ValueError("Positions must be 3D vectors (x, y, z)")
        
        #Compute error vector and norm
        error_norm = np.linalg.norm(pos_with_error - pos_no_error)
        
        #Return dictionary with detailed error information
        return error_norm

    def get_euler_angles(self, apply_errors=False):
        """Convert a rotation matrix to Euler angles (ZYX convention) in radians."""
        #Safeguard for matrix initialization
        if apply_errors and not hasattr(self, 'Matrix_e'):
            raise ValueError("Error matrix not computed. Call evaluate_param with apply_errors=True first.")
        if not apply_errors and not hasattr(self, 'Matrix'):
            raise ValueError("Nominal matrix not computed. Call evaluate_param with apply_errors=False first.")

        #Select rotation matrix
        rotation_matrix = self.Matrix_e[:3,:3] if apply_errors else self.Matrix[:3, :3]

        #Convert to numpy array and ensure 3x3
        rotation_matrix = np.array(rotation_matrix, dtype=np.float64)
        assert rotation_matrix.shape == (3, 3), "Rotation matrix must be 3x3"

        #Validate orthogonality and determinant
        if not np.allclose(np.dot(rotation_matrix.T, rotation_matrix), np.eye(3), atol=1e-6):
            print("Warning: Rotation matrix is not orthogonal")
        if not np.allclose(np.linalg.det(rotation_matrix), 1.0, atol=1e-6):
            print("Warning: Rotation matrix determinant is not 1")

        #Compute Euler angles (ZYX convention)
        sy = np.sqrt(rotation_matrix[0, 0]**2 + rotation_matrix[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            y = np.arctan2(-rotation_matrix[2, 0], sy)
            z = 0

        # Debug output
        #print(f"Debugging: Rotation matrix {'with errors' if apply_errors else 'without errors'}:\n", rotation_matrix)
        #print(f"Euler angles (radians): x={x}, y={y}, z={z}")

        return x, y, z  # Return angles in radians

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

    def plot_workspace(self, joint_ranges, fixed_values=None, num_samples=5000, apply_errors=False, title=None):
        """
        Plota o espaço de trabalho do mecanismo usando amostragem de Monte Carlo.
        Busca os valores fixos diretamente dos parâmetros (DH) do robô.
        
        :param joint_ranges: Dicionário mapeando os símbolos das juntas móveis para tuplas (min_val, max_val).
        :param fixed_values: (Opcional) Dicionário para adicionar ou sobrescrever valores estáticos.
        :param num_samples: Número de pontos aleatórios para gerar o espaço de trabalho.
        :param apply_errors: Booleano para aplicar ou não os erros ao espaço de trabalho.
        :param title: Título do gráfico.
        """
        # 1. Obtém a cinemática direta simbólica
        T, pos = self.forward_kinematics(apply_errors)
        
        # 2. Busca automaticamente os valores fixos da construção do robô (self.param)
        subs_dict = {}
        for i, params in enumerate(self.param):
            if 'a' in params: subs_dict[self.a[i]] = params['a']
            if 'alpha' in params: subs_dict[self.alpha[i]] = params['alpha']
            if 'd' in params: subs_dict[self.d[i]] = params['d']
            if 'theta' in params: subs_dict[self.theta[i]] = params['theta']
            
            # Trata offset para juntas prismáticas
            if params.get('type') == 'prismatic' and 'theta_offset' in params:
                subs_dict[self.theta[i]] = params['theta_offset']
                
            # Aplica ou zera os erros
            if apply_errors and 'errors' in params:
                errors = params['errors']
                if 'phi' in errors: subs_dict[self.phi[i]] = errors['phi']
                if 'epsilon' in errors: subs_dict[self.epsilon[i]] = errors['epsilon']
                if 'sigma' in errors: subs_dict[self.sigma[i]] = errors['sigma']
                if 'beta' in errors: subs_dict[self.beta[i]] = errors['beta']
            else:
                subs_dict[self.phi[i]] = 0
                subs_dict[self.epsilon[i]] = 0
                subs_dict[self.sigma[i]] = 0
                subs_dict[self.beta[i]] = 0

        # 3. Adiciona/sobrescreve com valores fixos adicionais caso fornecidos
        if fixed_values:
            subs_dict.update(fixed_values)
            
        # 4. Remove do subs_dict as juntas que vão variar (para mantê-las simbólicas na equação)
        varying_symbols = list(joint_ranges.keys())
        for sym in varying_symbols:
            if sym in subs_dict:
                del subs_dict[sym]
                
        if not varying_symbols:
            raise ValueError("Forneça pelo menos uma junta móvel no dicionário 'joint_ranges'.")

        # 5. Substitui os valores fixos no vetor de posição
        pos_expr = pos.subs(subs_dict)

        # 6. Converte as expressões simbólicas em funções rápidas do numpy
        x_func = sp.lambdify(varying_symbols, pos_expr[0], modules='numpy')
        y_func = sp.lambdify(varying_symbols, pos_expr[1], modules='numpy')
        z_func = sp.lambdify(varying_symbols, pos_expr[2], modules='numpy')
        
        # 7. Gera amostras aleatórias uniformes para as juntas no range definido
        random_inputs = []
        for sym in varying_symbols:
            min_val, max_val = joint_ranges[sym]
            random_inputs.append(np.random.uniform(min_val, max_val, num_samples))
            
        # 8. Avalia os pontos
        X = x_func(*random_inputs)
        Y = y_func(*random_inputs)
        Z = z_func(*random_inputs)
        
        # Converte constantes para arrays (caso algum eixo, como o Z em robôs planares, não mude)
        if np.isscalar(X): X = np.full(num_samples, X)
        if np.isscalar(Y): Y = np.full(num_samples, Y)
        if np.isscalar(Z): Z = np.full(num_samples, Z)
        
        # 9. Plota o espaço de trabalho
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Cor varia em Z para robôs 3D, ou em Y para robôs planares
        color_var = Z if np.ptp(Z) > 1e-5 else Y 
        scatter = ax.scatter(X, Y, Z, c=color_var, cmap='viridis', s=2, alpha=0.6)
        
        cbar = fig.colorbar(scatter, ax=ax, pad=0.1, shrink=0.7)
        cbar.set_label('Variação de Profundidade/Altura')
        
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(title if title else 'Espaço de Trabalho do Mecanismo')
        
        # Ajusta os eixos proporcionalmente e evita quebra de limite em robôs planares (2D)
        dx = X.max() - X.min()
        dy = Y.max() - Y.min()
        dz = Z.max() - Z.min()
        max_range = max(dx, dy, dz) / 2.0
        if max_range == 0: max_range = 10 # Prevenção de erro caso o range seja nulo
        
        mid_x = (X.max() + X.min()) * 0.5
        mid_y = (Y.max() + Y.min()) * 0.5
        mid_z = (Z.max() + Z.min()) * 0.5
        
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()