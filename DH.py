"""
Code for the Denavit-Hartenberg (DH) parameters, forward kinematics, and visualization.
Refactored for cleaner architecture (Separation of Concerns).

Parameters Input example:
    Dh_param = [
        {'type': 'revolute', 'a':0, 'alpha': 0, 'd': 0,
         'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0}},
        {'type': 'prismatic', 'a':150, 'alpha': 0, 'theta_offset': 0,
         'errors': {'sigma': 0, 'beta': 0, 'epsilon': 0, 'phi': 0}}, 
    ]

Usage:
    robot = Mechanism(Dh_param)
    plotter = MechanismPlotter(robot)
    
    # Cálculos
    T, pos = robot.forward_kinematics(apply_errors=False)
    T_num, pos_num, rot_num = robot.evaluate_param(T, variable_values)
    
    # Visualização
    plotter.plot_mechanism(variable_values, title='My Robot', initial_config=False)
"""

import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from typing import List, Dict, Optional, Tuple, Union

class Mechanism:
    """Classe responsável puramente pela modelagem algébrica e numérica do robô."""
    
    def __init__(self, param: List[Dict]):
        self.param = param
        self.n_joints = len(param)

        # Definição dos símbolos (Parâmetros DH)
        self.a = [sp.Symbol(f'a_{i}') for i in range(self.n_joints)]
        self.alpha = [sp.Symbol(f'alpha_{i}') for i in range(self.n_joints)]
        self.d = [sp.Symbol(f'd_{i}') for i in range(self.n_joints)]
        self.theta = [sp.Symbol(f'theta_{i}') for i in range(self.n_joints)]
        
        # Definição dos símbolos (Erros)
        self.phi = [sp.Symbol(f'phi_{i}') for i in range(self.n_joints)]
        self.epsilon = [sp.Symbol(f'epsilon_{i}') for i in range(self.n_joints)]
        self.sigma = [sp.Symbol(f'sigma_{i}') for i in range(self.n_joints)]
        self.beta = [sp.Symbol(f'beta_{i}') for i in range(self.n_joints)]

        # Conjunto de validação rápida
        self.valid_symbols = set(self.a + self.alpha + self.d + self.theta +
                                 self.phi + self.epsilon + self.sigma + self.beta)

    def dh_matrix(self, i: int, apply_errors: bool = False) -> sp.Matrix:
        """Retorna a matriz de transformação homogênea simbólica de um único elo."""
        a = self.a[i] + (self.sigma[i] if apply_errors else 0)
        alpha = self.alpha[i] + (self.beta[i] if apply_errors else 0)
        d = self.d[i] + (self.epsilon[i] if apply_errors else 0)
        theta = self.theta[i] + (self.phi[i] if apply_errors else 0)
        
        cos_th, sin_th = sp.cos(theta), sp.sin(theta)
        cos_al, sin_al = sp.cos(alpha), sp.sin(alpha)
        
        return sp.Matrix([
            [cos_th, -sin_th * cos_al,  sin_th * sin_al, a * cos_th],
            [sin_th,  cos_th * cos_al, -cos_th * sin_al, a * sin_th],
            [     0,           sin_al,           cos_al,          d],
            [     0,                0,                0,          1]
        ])
    
    def forward_kinematics(self, apply_errors: bool = False) -> Tuple[sp.Matrix, sp.Matrix]:
        """Calcula a cinemática direta simbólica (Matriz T e Vetor Posição)."""
        T = sp.eye(4)
        for i in range(self.n_joints):
            T = T * self.dh_matrix(i, apply_errors)
            
        T = sp.simplify(T)
        position = T[:3, 3]
        return T, position

    def inverse_kinematics(self, target_position: List[float], initial_guess: Optional[Dict] = None, fixed_values: Optional[Dict] = None, apply_errors: bool = False, tolerance: float = 1e-6) -> Dict:
        """
        Calcula a Cinemática Inversa numericamente usando otimização (SciPy).
        Busca os valores das juntas que minimizam a distância até a posição alvo.
        
        :param target_position: Lista ou array [x, y, z] com a posição alvo do efetuador.
        :param initial_guess: Dicionário com o chute inicial para as juntas móveis. Ajuda a evitar singularidades.
        :param fixed_values: Dicionário com os valores estáticos (se houver).
        :param apply_errors: Booleano para aplicar ou não os erros DH ao cálculo.
        :param tolerance: Tolerância de erro para aceitar a solução.
        :return: Dicionário contendo os valores numéricos otimizados para as juntas móveis.
        """
        # 1. Identificar as juntas variáveis (móveis)
        varying_symbols = []
        for i, params in enumerate(self.param):
            sym = self.d[i] if params.get('type') == 'prismatic' else self.theta[i]
            varying_symbols.append(sym)

        # 2. Obter o vetor de posição simbólico da Cinemática Direta
        _, pos_sym = self.forward_kinematics(apply_errors)

        # 3. Construir o dicionário base com os parâmetros construtivos
        subs_dict = self._build_subs_dict(fixed_values, apply_errors)

        # 4. Remover as variáveis móveis do dicionário para que permaneçam simbólicas
        for sym in varying_symbols:
            if sym in subs_dict:
                del subs_dict[sym]

        # 5. Substituir tudo o que é fixo na expressão de posição
        pos_expr = pos_sym.subs(subs_dict)

        # 6. Converter as expressões simbólicas para funções numéricas Numpy (acelera a otimização)
        pos_func_x = sp.lambdify(varying_symbols, pos_expr[0], modules='numpy')
        pos_func_y = sp.lambdify(varying_symbols, pos_expr[1], modules='numpy')
        pos_func_z = sp.lambdify(varying_symbols, pos_expr[2], modules='numpy')

        target = np.array(target_position, dtype=float)

        # 7. Função objetivo: Minimizar a distância Euclidiana ao quadrado
        def objective(q):
            current_pos = np.array([pos_func_x(*q), pos_func_y(*q), pos_func_z(*q)])
            error = current_pos - target
            return np.sum(error**2)

        # 8. Configurar o ponto de partida (chute inicial)
        if initial_guess:
            q0 = [initial_guess.get(sym, 0.0) for sym in varying_symbols]
        else:
            q0 = [0.1] * len(varying_symbols) # 0.1 para evitar singularidades comuns no ponto 0

        # 9. Otimização não linear (BFGS)
        result = minimize(objective, q0, method='BFGS', tol=tolerance)

        if result.fun > 1e-3:
            print(f"Aviso: A Cinemática Inversa pode não ter atingido o ponto exato. Erro residual: {np.sqrt(result.fun):.4f} mm")

        # 10. Mapear o resultado numérico de volta para os símbolos das juntas
        return {sym: float(val) for sym, val in zip(varying_symbols, result.x)}
    
    def _build_subs_dict(self, variable_values: Optional[Dict] = None, apply_errors: bool = False) -> Dict:
        """Centraliza e limpa a lógica de montagem do dicionário de substituição."""
        subs_dict = {}
        
        for i, params in enumerate(self.param):
            # Extração limpa usando .get() com valores padrão (0)
            subs_dict[self.a[i]] = params.get('a', 0)
            subs_dict[self.alpha[i]] = params.get('alpha', 0)
            subs_dict[self.d[i]] = params.get('d', 0)
            
            if params.get('type') == 'prismatic':
                subs_dict[self.theta[i]] = params.get('theta_offset', 0)
            else:
                subs_dict[self.theta[i]] = params.get('theta', 0)
            
            # Tratamento de erros
            errors = params.get('errors', {}) if apply_errors else {}
            subs_dict[self.phi[i]] = errors.get('phi', 0)
            subs_dict[self.epsilon[i]] = errors.get('epsilon', 0)
            subs_dict[self.sigma[i]] = errors.get('sigma', 0)
            subs_dict[self.beta[i]] = errors.get('beta', 0)

        # Sobrescreve com valores dinâmicos do usuário
        if variable_values:
            for symbol, value in variable_values.items():
                if symbol in self.valid_symbols:
                    subs_dict[symbol] = value
                    
        return subs_dict
    
    def evaluate_param(self, T: Optional[sp.Matrix] = None, variable_values: Optional[Dict] = None, apply_errors: bool = False) -> Tuple[Union[sp.Matrix, np.ndarray], Union[sp.Matrix, np.ndarray], Union[sp.Matrix, np.ndarray]]:
        """Aplica os valores numéricos na matriz de transformação."""
        if T is None or not isinstance(T, sp.Matrix):
            T, _ = self.forward_kinematics(apply_errors)
           
        subs_dict = self._build_subs_dict(variable_values, apply_errors)
        T_num = T.subs(subs_dict)
        
        if variable_values is not None:
            T_eval = np.array(T_num, dtype=float)
            return T_eval, T_eval[:3, 3], T_eval[:3, :3]
            
        return T_num, T_num[:3, 3], T_num[:3, :3] 

    @staticmethod
    def evaluate_error(pos_no_error: np.ndarray, pos_with_error: np.ndarray) -> float:
        """Calcula a norma do erro (distância) entre duas posições numéricas 3D."""
        pos_no_error = np.array(pos_no_error, dtype=float).flatten()
        pos_with_error = np.array(pos_with_error, dtype=float).flatten()
        
        if len(pos_no_error) != 3 or len(pos_with_error) != 3:
            raise ValueError("Positions must be 3D vectors (x, y, z)")
            
        return float(np.linalg.norm(pos_with_error - pos_no_error))

    @staticmethod
    def get_euler_angles(rotation_matrix: np.ndarray) -> Tuple[float, float, float]:
        """Converte uma matriz de rotação em ângulos de Euler (convenção ZYX)."""
        rot_matrix = np.array(rotation_matrix, dtype=np.float64)
        if rot_matrix.shape != (3, 3):
            raise ValueError("Rotation matrix must be 3x3")

        sy = np.sqrt(rot_matrix[0, 0]**2 + rot_matrix[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(rot_matrix[2, 1], rot_matrix[2, 2])
            y = np.arctan2(-rot_matrix[2, 0], sy)
            z = np.arctan2(rot_matrix[1, 0], rot_matrix[0, 0])
        else:
            x = np.arctan2(-rot_matrix[1, 2], rot_matrix[1, 1])
            y = np.arctan2(-rot_matrix[2, 0], sy)
            z = 0

        return float(x), float(y), float(z)


class MechanismPlotter:
    """Classe responsável exclusivamente por gráficos e plotagem do robô."""
    
    def __init__(self, mechanism: Mechanism):
        self.mech = mechanism

    def get_joint_positions(self, variable_values: Dict, apply_errors: bool = False) -> np.ndarray:
        """Retorna as posições (X,Y,Z) de cada junta para desenhar os elos."""
        positions = [[0, 0, 0]]
        T = sp.eye(4)
        
        for i in range(self.mech.n_joints):
            T = T * self.mech.dh_matrix(i, apply_errors)
            _, pos_eval, _ = self.mech.evaluate_param(T, variable_values, apply_errors)
            positions.append(pos_eval)
            
        try:
            return np.array([[float(coord) for coord in pos] for pos in positions])
        except TypeError:
            raise TypeError("Símbolos algébricos pendentes. Impossível plotar gráfico.")

    def plot_mechanism(self, variable_values: Optional[Dict] = None, title: Optional[str] = None, initial_config: bool = False, plot_type: str = '3d'):
        """Plota o mecanismo em 2D ou 3D."""
        if plot_type not in ['2d', '3d']:
            raise ValueError("plot_type must be '2d' or '3d'")

        fig = plt.figure(figsize=(10, 8)) if plot_type == '3d' else plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d') if plot_type == '3d' else fig.add_subplot(111)

        # Calculando posições nominais ("zeradas" / config inicial)
        positions_initial = self.get_joint_positions({}, apply_errors=False)

        if initial_config:
            # Se pedido a config inicial, mas com valores passados, usamos eles
            if variable_values:
                positions_initial = self.get_joint_positions(variable_values, apply_errors=False)
                
            if plot_type == '3d':
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], positions_initial[:, 2], 'm-o', label='Initial Config')
            else:
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], 'm-o', label='Initial Config')
        else:
            if not variable_values:
                raise ValueError("variable_values é necessário para plotar fora da initial_config.")
                
            joints_no_error = self.get_joint_positions(variable_values, apply_errors=False)
            joints_with_error = self.get_joint_positions(variable_values, apply_errors=True)
            
            pos_no = joints_no_error[-1]
            pos_err = joints_with_error[-1]
            error_vec = pos_err - pos_no

            if plot_type == '3d':
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], positions_initial[:, 2], 'm-o', alpha=0.3, label='Initial')
                ax.plot(joints_no_error[:, 0], joints_no_error[:, 1], joints_no_error[:, 2], 'b-o', label='Nominal')
                ax.plot(joints_with_error[:, 0], joints_with_error[:, 1], joints_with_error[:, 2], 'r--o', label='With Errors')
                ax.quiver(pos_no[0], pos_no[1], pos_no[2], error_vec[0], error_vec[1], error_vec[2], color='g', label='Error Vector')
            else:
                ax.plot(positions_initial[:, 0], positions_initial[:, 1], 'm-o', alpha=0.3, label='Initial')
                ax.plot(joints_no_error[:, 0], joints_no_error[:, 1], 'b-o', label='Nominal')
                ax.plot(joints_with_error[:, 0], joints_with_error[:, 1], 'r--o', label='With Errors')
                ax.quiver(pos_no[0], pos_no[1], error_vec[0], error_vec[1], color='g', angles='xy', scale_units='xy', scale=1, label='Error Vector')

        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        if plot_type == '3d':
            ax.set_zlabel('Z (mm)')
            ax.view_init(elev=20, azim=45)
        else:
            ax.axis('equal')
            ax.grid(True)
            
        ax.set_title(title or f'Mechanism Plot ({plot_type.upper()})')
        ax.legend()
        plt.show()

    def plot_workspace(self, joint_ranges: Dict, fixed_values: Optional[Dict] = None, num_samples: int = 5000, apply_errors: bool = False, title: Optional[str] = None):
        """Plota o espaço de trabalho usando Monte Carlo."""
        T, pos = self.mech.forward_kinematics(apply_errors)
        
        # Pega a base dos valores do mecanismo
        subs_dict = self.mech._build_subs_dict(fixed_values, apply_errors)

        # Remove as variáveis móveis para mantê-las simbólicas
        varying_symbols = list(joint_ranges.keys())
        for sym in varying_symbols:
            if sym in subs_dict:
                del subs_dict[sym]

        if not varying_symbols:
            raise ValueError("Forneça pelo menos uma junta móvel em 'joint_ranges'.")

        pos_expr = pos.subs(subs_dict)

        x_func = sp.lambdify(varying_symbols, pos_expr[0], modules='numpy')
        y_func = sp.lambdify(varying_symbols, pos_expr[1], modules='numpy')
        z_func = sp.lambdify(varying_symbols, pos_expr[2], modules='numpy')
        
        random_inputs = [np.random.uniform(joint_ranges[sym][0], joint_ranges[sym][1], num_samples) for sym in varying_symbols]
        
        X, Y, Z = x_func(*random_inputs), y_func(*random_inputs), z_func(*random_inputs)
        
        if np.isscalar(X): X = np.full(num_samples, X)
        if np.isscalar(Y): Y = np.full(num_samples, Y)
        if np.isscalar(Z): Z = np.full(num_samples, Z)
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        color_var = Z if np.ptp(Z) > 1e-5 else Y 
        scatter = ax.scatter(X, Y, Z, c=color_var, cmap='viridis', s=2, alpha=0.6)
        fig.colorbar(scatter, ax=ax, pad=0.1, shrink=0.7).set_label('Variação Espacial')
        
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(title or 'Workspace Analysis (Monte Carlo)')
        
        # Ajusta eixos isométricos
        max_range = max(X.max() - X.min(), Y.max() - Y.min(), Z.max() - Z.min()) / 2.0
        if max_range == 0: max_range = 10 
        
        mid_x, mid_y, mid_z = (X.max() + X.min()) / 2, (Y.max() + Y.min()) / 2, (Z.max() + Z.min()) / 2
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()

    def simulate_movement(self, start_values: Dict, end_values: Dict, frames: int = 60, interval: int = 50, plot_type: str = '3d', title: Optional[str] = None):
        """
        Simula e anima a movimentação do robô entre duas configurações de juntas.
        Retorna um player de vídeo em HTML5 nativo para Jupyter Notebook.
        """
        import matplotlib.animation as animation
        from IPython.display import HTML, display
        
        if plot_type not in ['2d', '3d']:
            raise ValueError("plot_type must be '2d' ou '3d'")

        # 1. Gerar os quadros de interpolação (trajetória linear nas juntas)
        interpolated_values = []
        for i in range(frames):
            current_vals = {}
            for sym in start_values.keys():
                start = start_values[sym]
                end = end_values.get(sym, start)
                # Interpola de start até end
                current_vals[sym] = start + (end - start) * (i / (frames - 1))
            interpolated_values.append(current_vals)

        # 2. Pré-calcular todas as posições para ajustar os limites do gráfico sem distorcer
        all_positions = [self.get_joint_positions(vals, apply_errors=False) for vals in interpolated_values]
        
        all_x = np.concatenate([pos[:, 0] for pos in all_positions])
        all_y = np.concatenate([pos[:, 1] for pos in all_positions])
        
        fig = plt.figure(figsize=(10, 8)) if plot_type == '3d' else plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d') if plot_type == '3d' else fig.add_subplot(111)

        # Configurar limites da câmera baseados no movimento total
        margin = 20 # Margem de respiro visual
        ax.set_xlim(all_x.min() - margin, all_x.max() + margin)
        ax.set_ylim(all_y.min() - margin, all_y.max() + margin)
        
        if plot_type == '3d':
            all_z = np.concatenate([pos[:, 2] for pos in all_positions])
            z_ptp = np.ptp(all_z) if np.ptp(all_z) > 0 else 100 # Previne erros se o Z for constante
            mid_z = np.mean(all_z)
            ax.set_zlim(mid_z - z_ptp/2 - margin, mid_z + z_ptp/2 + margin)
            ax.set_zlabel('Z (mm)')
            ax.view_init(elev=20, azim=45)
        else:
            ax.axis('equal') # Mantém a proporção real 1:1 no plano 2D
            ax.grid(True)
            
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title(title or f'Simulação de Movimento ({plot_type.upper()})')

        # 3. Inicializar a linha que representará os elos do robô
        if plot_type == '3d':
            line, = ax.plot([], [], [], 'b-o', linewidth=4, markersize=8)
        else:
            line, = ax.plot([], [], 'b-o', linewidth=4, markersize=8)

        # Posição alvo fantasma (Tracejada vermelha)
        target_pos = all_positions[-1]
        if plot_type == '3d':
            ax.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2], 'r--', alpha=0.5, label='Alvo Desejado')
        else:
            ax.plot(target_pos[:, 0], target_pos[:, 1], 'r--', alpha=0.5, label='Alvo Desejado')
        ax.legend()

        # 4. Função de atualização injetada em cada frame do vídeo
        def update(frame):
            positions = all_positions[frame]
            if plot_type == '3d':
                line.set_data(positions[:, 0], positions[:, 1])
                line.set_3d_properties(positions[:, 2])
            else:
                line.set_data(positions[:, 0], positions[:, 1])
            return line,

        # 5. Criar Animação
        ani = animation.FuncAnimation(fig, update, frames=frames, interval=interval, blit=False)
        
        # Fechar a figura estática em background para não poluir o Jupyter
        plt.close(fig)
        
        # Injetar o Player HTML5 interativo na célula do notebook
        return display(HTML(ani.to_jshtml()))