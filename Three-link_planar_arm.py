import sympy as sp
import DH 

if __name__ == "__main__":
    
    #Define the DH parameters:(Three-link planar arm)
    Three_link_planar_arm = [
        {'type': 'revolute', 'a': 80, 'alpha': 0, 'd': 5, 
         'errors': {'sigma': 0.5, 'beta': 0.017453 ,}},

        {'type': 'revolute', 'a': 80, 'alpha': 0, 'd': 0,
         'errors': {'sigma': 0.5, 'beta': 0.017453 ,}},
         
        {'type': 'revolute', 'a': 0, 'alpha': 0, 'd': 0, 
        'errors': {'sigma': 0.5, 'beta': 0.017453 ,}},
    ]
    
    #Create the robot:
    robot = DH.Mechanism(Three_link_planar_arm)
        
    #To plot the robot, we need to provide the values of the variables. including other parameters that are not theta
    variable_values = {
    #Variables not provided firstly:
    
        
    #Variables of movement:
    robot.theta[0]: (sp.pi/12),   #theta_0 for the first cylindrical joint 15 degrees
    robot.theta[1]: (sp.pi/4),    #theta_1 for the second cylindrical joint 45 degrees
    robot.theta[2]: (sp.pi/3),    #theta_2 for the third cylindrical joint 60 degrees

    #Variables of errors:
    robot.epsilon[0]: 0.0,       #epsilon_0 for the first cylindrical joint
    robot.epsilon[1]: 0.0,       #epsilon_1 for the second cylindrical joint
    robot.epsilon[2]: 0.0,       #epsilon_2 for the third cylindrical joint
    robot.phi[0]: 0.0,           #phi_0 for the first cylindrical joint
    robot.phi[1]: 0.0,           #phi_1 for the second cylindrical joint
    robot.phi[2]: 0.0,           #phi_2 for the third cylindrical joint
    } 

    robot.plot_mechanism( title ='Manipulador em Estado Inicial',initial_config=True, plot_type= '2d') 

    """Begin the calculations algebrically"""

    #Calculate the forward kinematics without errors:
    matrix_g,position_g=robot.forward_kinematics(False)
    print("\nMatrix and position algebrical without errors:\n")
    print("\nMatrix:\n", matrix_g)
    print("\nPosition:\n", position_g)
    
    
    #Calculate the forward kinematics with errors:
    matrix_g_e,position_g_e=robot.forward_kinematics(True)
    print("\nMatrix and position algebrical with errors:\n")
    print("Matrix:\n", matrix_g_e)
    print("\nPosition:\n", position_g_e)

    """Begin the calculations numerically:"""

    #Calculate the forward kinematics without errors DH parameters applied:
    matrix_g_numerical, position_g_numerical = robot.evaluate_param(matrix_g)
    print("\nMatrix and position numerical without errors:(Book Validation)\n")
    print("\nMatrix:\n", matrix_g_numerical)
    print("\nPosition:\n", position_g_numerical)

    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_ge_numerical, position_ge_numerical = robot.evaluate_param(matrix_g_e,apply_errors=True)
    print("\nMatrix and position numerical with errors:(Book Validation)\n")
    print("\nMatrix:\n", matrix_ge_numerical)
    print("\nPosition:\n", position_ge_numerical)
    
    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_numerical, position_numerical = robot.evaluate_param(matrix_g,variable_values)
    print("\nMatrix and position numerical without errors:\n")
    print("\nMatrix:\n", matrix_numerical)
    print("\nPosition:\n", position_numerical)

    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_numerical_e, position_numerical_e = robot.evaluate_param(matrix_g_e, variable_values, apply_errors=True)
    print("\nMatrix and position numerical with errors:\n")
    print("Matrix:\n", matrix_numerical_e)
    print("\nPosition:\n", position_numerical_e)

    #Calculate error values:
    print("\n [X, Y, Z] position values:")
    print("Position without erros: \n", [p for p in position_numerical])
    print("Position with errors: \n", [p for p in position_numerical_e])
    error=robot.evaluate_error(position_numerical, position_numerical_e)
    print("\n Error values: \n", error)
        
    #Plot the robot:
    #CERTIFY ALL THE VARIABLES ARE PROVIDED BEFORE PLOTTING#
    robot.plot_mechanism(variable_values, title ='Mechanism kinematics with and without errors', plot_type= '2d')
    