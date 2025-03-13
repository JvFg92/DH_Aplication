import sympy as sp
import DH 

if __name__ == "__main__":
    
    #Define the DH parameters:(exempli gratia: Anthropomorphic Arm)
    Anthropomorphic_Arm  = [
        {'type': 'revolute',  'alpha': sp.pi/2, 'd': 0.0,
         'errors': {'sigma': 0.01, 'epsilon': 0.02, 'beta': 0.0003, 'phi': 0.001,}},

        {'type': 'revolute',  'alpha': 0, 'd': 0, 
         'errors': {'sigma': 0.05, 'epsilon': 0.04, 'beta': 0.0006, 'phi': 0.005,}},

        {'type': 'revolute', 'a':1, 'alpha': 0, 'd': 0, 
         'errors': {'sigma': 0.1, 'epsilon': 0.06, 'beta': 0.0009, 'phi': 0.015,}}
    ]

    #Create the robot:
    robot = DH.Mechanism(Anthropomorphic_Arm)

    #Evaluate the position of the effector using optional variable values:
    """To plot the robot, we need to provide the values of the variables. including other parameters that are not theta"""
    variable_values = {
    #Variables not provided (Defined in the DH parameters, for plot eg):
    robot.a[0]:     5,
    robot.a[1]:     10,
    robot.a[2]:     15,
    #Variables of movement:
    robot.theta[0]: 0.5,    #theta_0 for the first cylindrical joint
    robot.theta[1]: sp.pi/4,    #theta_1 for the second cylindrical joint   
    robot.theta[2]: sp.pi/6,     #theta_2 for the third cylindrical joint
    }

    #robot.plot_mechanism(variable_values, title ='Mechanism kinematics without errors',initial_config=True)

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
    
    #Validating the matrix:
    for i in range(matrix_g_numerical.shape[0]):
        for j in range(matrix_g_numerical.shape[1]):
            print(f"\n Element ({i},{j}):", matrix_g_numerical[i,j], end=" ")
    print("\n")

    print("\nPosition:\n", position_g_numerical)

    #Calculate the forward kinematics with errors DH parameters applied:
    matrix_g_numerical_e, position_g_numerical_e = robot.evaluate_param(matrix_g_e, apply_errors=True)
    print("\nMatrix and position numerical with errors:\n")
    print("Matrix:\n", matrix_g_numerical_e)
    print("\nPosition:\n", position_g_numerical_e)

    """Start calculations numerically with optional variable values ​​(graphical example):"""

    #Calculate the forward kinematics without errors DH parameters applied:
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
    robot.plot_mechanism(variable_values, title ='Mechanism kinematics with and without errors')
    