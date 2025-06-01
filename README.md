# 🤖 Denavit-Hartenberg Method in Python for Manipulator Robot Analysis 🐍

A project focused on dimensional and error analysis in manipulator robots using the Denavit-Hartenberg (DH) method implemented in Python.

⚙️ Development Files
- `DH.py`:
*Calculates direct kinematics (with and without geometric errors applied to joints) algebraically and numerically.
*Computes trajectory error.
*Defines end-effector coordinates and orientation.
*Determines Euler angles for orientation.
*Generates graphical plots of the mechanism after joint movement.

- `Anthropomorphic_Arm.py`:
*Contains the primary validation of the DH method.

- `Spherical_Arm.py`:
*Provides secondary validation using a spherical arm model.

- `Three-link_planar_arm.py`:
*Demonstrates the application of the DH method to a three-link planar arm model for research purposes.

The development of this method was validated using an anthropomorphic arm with 3 cylindrical joints (see Image 1: Anthropomorphic Arm). The direct kinematics results align with established bibliography (see Results section). ✅

📝 Note: This project may receive further updates and additions as needed.

### **🇬🇧 EN / 🇧🇷 PT :** 
# 🇧🇷 Desenvolvimento do método Denavit-Hartenberg em Python, em um projeto de análise dimensional e de erros em robôs manipuladores!

Um projeto focado na análise dimensional e de erros em robôs manipuladores utilizando o método Denavit-Hartenberg (DH) implementado em Python.

⚙️ Arquivos de Desenvolvimento
- `DH.py`:
*Calcula a cinemática direta (com e sem erros geométricos aplicados às juntas) algebricamente e numericamente.
*Calcula o erro de trajetória.
*Define as coordenadas e a orientação do efetor final.
*Determina ângulos de Euler para orientação.
*Gera gráficos do mecanismo após o movimento da junta.

- `Anthropomorphic_Arm.py`:
*Contém a validação primária do método DH.

- `Spherical_Arm.py`:
*Fornece validação secundária utilizando um modelo de braço esférico.

- `Three-link_planar_arm.py`:
*Demonstra a aplicação do método DH a um modelo de braço plano de três elos para fins de pesquisa.

O desenvolvimento deste método foi validado utilizando um braço antropomórfico com 3 juntas cilíndricas (ver Imagem 1: Braço Antropomórfico). Os resultados da cinemática direta estão alinhados com a bibliografia estabelecida (ver seção Resultados). ✅

📝 Observação: Este projeto pode receber atualizações e adições conforme necessário.
Enviar feedback
Resultados de tradução disponíveis



## 💪**Antropomorphic Arm**

![image](https://github.com/user-attachments/assets/fb2276c0-c112-48a9-955f-f4f296786b1b)

*Avaliable at*: [Robotics: Modelling, Planning and Control; Sciavicco, et al. 2009](https://link.springer.com/book/10.1007/978-1-84628-642-1)


## 📊 *Resultados / Results:

📚 **Bibliografia / bibliography:**
Parâmetros Denavit-Hartenberg base:

$$
T = \begin{bmatrix}
\cos(\theta_0)\cos(\theta_1 + \theta_2) & -\cos(\theta_0)\sin(\theta_1 + \theta_2) & \sin(\theta_0) & a_1\cos(\theta_0)\cos(\theta_1) + a_2\cos(\theta_0)\cos(\theta_1 + \theta_2) \\
\sin(\theta_0)\cos(\theta_1 + \theta_2) & -\sin(\theta_0)\sin(\theta_1 + \theta_2) & -\cos(\theta_0) & a_1\sin(\theta_0)\cos(\theta_1) + a_2\sin(\theta_0)\cos(\theta_1 + \theta_2) \\
\sin(\theta_1 + \theta_2) & \cos(\theta_1 + \theta_2) & 0 & a_1\sin(\theta_1) + a_2\sin(\theta_1 + \theta_2) \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

## **Used Parameters / *Parâmetros utilizados*:**

| Joint (i) | θᵢ (rad) | dᵢ (mm)  | aᵢ (mm) | αᵢ (rad) |
|-----------|---------:|--------:|-------:|---------:|
| 1         |   0.5    |    0   |   5    |    $\pi/2$     |
| 2         |   $\pi/4$    |    0   |   10    |    0     |
| 3         |   $\pi/6$     |    0   |   15    |    0     |



| Joint (i) | $\phi$ᵢ (rad) | $\epsilon$ᵢ (mm)  | $\sigma$ᵢ (mm) | $\beta$ᵢ (rad) |
|-----------|---------:|--------:|-------:|---------:|
| 1         |   0.001    |    0.02   |   0.01    |    0.0003     |
| 2         |   0.005    |    0.04   |   0.05    |    0.0006     |
| 3         |   0.015    |    0.06   |   0.10    |    0.0009     |

## **Trajetória calculada / *Calculated trajectory*:**

![image](https://github.com/user-attachments/assets/903c9679-f460-4f5c-b5fe-55307f81f451)

## **Kinematic Transformation Matrix / *Matriz Transformação Cinemática*:**

### **Withot errors / *Sem erros*:**

$$
A' = \begin{bmatrix}
0.22713508 & -0.84767966 & 0.47942554 & 14.00038483  \\
0.12408446 & -0.46308951 & -0.87758256 & 7.64844509 \\
0.96592583 & 0.25881905 & 0        & 21.55995521  \\
0 & 0 & 0 & 1
\end{bmatrix}
$$


### ⚠️ **With errors / *Com erros*:**

$$
A'' = \begin{bmatrix}
2.10310063e-01  & -8.50871689e-01 &  4.81442671e-014 & 1.38208681e+01  \\
1.14483169e-01  & -4.67633968e-01 & -8.76477082e-01  & 7.44156283e+00  \\
9.70908482e-01  &  2.39449033e-01 & -9.37641832e-04  & 2.18225287e+01  \\
0 & 0 & 0 & 1
\end{bmatrix}
$$


## 📏**Calculated Modular Error / *Erro Modular Calculado*:**
Error = 0.3794356098314992 mm

## 🧭**Euler Anlges / *Ângulos de Euler*:**

  Yaw (X),  Pitch (Y),  Roll (Z)
-  Without errors:  $(1.5707963267948966, -1.30899693899575, 0.500000000000000)$

-  ⚠️ With errors:  $(1.57471213729465, -1.32899609966771, 0.498498303670928)$

### 🎓 Referência usada / Reference used: 
[Robotics: Modelling, Planning and Control; Sciavicco, et al. 2009](https://link.springer.com/book/10.1007/978-1-84628-642-1)

* 🇧🇷 O acesso ao material é provido pela Universidade Federal de Santa Catarina (UFSC)
* 🇬🇧  The access is provided by Federal University of Santa Catarina (UFSC)
