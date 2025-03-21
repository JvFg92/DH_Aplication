# Desenvolvimento do método Denavit-Hartenberg em Python, em um projeto de análise dimensional e de erros em robôs manipuladores!

Desenvolvimento:
- DH.py:
  - Cálculo da cinemática direta (com e sem erros de geometria aplicados às juntas) algebricamente e numericamente;
  - Cálculo de erro de trajetória;
  - Definição das coordenadas do efetuador final;
  - Plotagem gráfica do mecanismo após movimentação das juntas;

- Anthropomorphic_Arm.py:
  - Arquivo contendo a validação do método, descrita em sequência;

- Spherical_Arm.py:
  - Arquivo contendo validação secundária com um braço esférico;

- Three-link_planar_arm.py:
  - Arquivo contendo o a utilização do método em um elo modelo para pesquisa.

O desenvolvimento do método foi validado utilizando um braço antropomórfico com 3 juntas cilíndricas (Imagem 1 - Braço Antropomórfico), onde a cinemática direta coincidiu com o esperado na bibliografia (Resultados)!

*Nota: Novas adições podem acontecer de acordo com a necessidade de uso.

### **PT/EN:** 
# Development of the Denavit-Hartenberg method in Python, in a project of dimensional and error analysis in manipulator robots!

Development:
- DH.py:
  - Calculation of direct kinematics (with and without geometry errors applied to the joints) algebraically and numerically;
  - Trajectory error calculation;
  - Definition of end effector coordinates;
  - Graphical plotting of the mechanism after joint movement;

- Anthropomorphic_Arm.py:
  - File containing the validation of the method, described in sequence;

- Spherical_Arm.py:
  - File containing secondary validation with a spherical arm;

- Three-link_planar_arm.py:
  - File containing the use of the method in a model link for research.
 
The development of the method was validated using an anthropomorphic arm with 3 cylindrical joints (Image 1 - Anthropomorphic Arm), where the direct kinematics coincided with what was expected in the bibliography (Results)!

*Note: New additions may occur according to usage needs.


## **Antropomorphic Arm**

![image](https://github.com/user-attachments/assets/fb2276c0-c112-48a9-955f-f4f296786b1b)

Disponível em / *Avaliable at*: [Robotics: Modelling, Planning and Control; Sciavicco, et al. 2009](https://link.springer.com/book/10.1007/978-1-84628-642-1)


## *Resultados / Results:

**Bibliografia / bibliography:**
Parâmetros Denavit-Hartenberg base:



$$
T = \begin{bmatrix}
\cos(\theta_0)\cos(\theta_1 + \theta_2) & -\cos(\theta_0)\sin(\theta_1 + \theta_2) & \sin(\theta_0) & a_1\cos(\theta_0)\cos(\theta_1) + a_2\cos(\theta_0)\cos(\theta_1 + \theta_2) \\
\sin(\theta_0)\cos(\theta_1 + \theta_2) & -\sin(\theta_0)\sin(\theta_1 + \theta_2) & -\cos(\theta_0) & a_1\sin(\theta_0)\cos(\theta_1) + a_2\sin(\theta_0)\cos(\theta_1 + \theta_2) \\
\sin(\theta_1 + \theta_2) & \cos(\theta_1 + \theta_2) & 0 & a_1\sin(\theta_1) + a_2\sin(\theta_1 + \theta_2) \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Essa matriz usa a forma simplificada.

*This matrix uses the simplified form.*

**Gerada pelo código / Generated by code:**

$$
T' = \begin{bmatrix}
-\sin(\theta_1)\sin(\theta_2)\cos(\theta_0) + \cos(\theta_0)\cos(\theta_1)\cos(\theta_2) & -\sin(\theta_1)\cos(\theta_0)\cos(\theta_2) - \sin(\theta_2)\cos(\theta_0)\cos(\theta_1) & \sin(\theta_0) & a_1\cos(\theta_0)\cos(\theta_1) - a_2\sin(\theta_1)\sin(\theta_2)\cos(\theta_0) + a_2\cos(\theta_0)\cos(\theta_1)\cos(\theta_2) \\
-\sin(\theta_0)\sin(\theta_1)\sin(\theta_2) + \sin(\theta_0)\cos(\theta_1)\cos(\theta_2) & -\sin(\theta_0)\sin(\theta_1)\cos(\theta_2) - \sin(\theta_0)\sin(\theta_2)\cos(\theta_1) & -\cos(\theta_0) & a_1\sin(\theta_0)\cos(\theta_1) - a_2\sin(\theta_0)\sin(\theta_1)\sin(\theta_2) + a_2\sin(\theta_0)\cos(\theta_1)\cos(\theta_2) \\
\sin(\theta_1)\cos(\theta_2) + \sin(\theta_2)\cos(\theta_1) & -\sin(\theta_1)\sin(\theta_2) + \cos(\theta_1)\cos(\theta_2) & 0 & a_1\sin(\theta_1) + a_2\sin(\theta_1)\cos(\theta_2) + a_2\sin(\theta_2)\cos(\theta_1) \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Essa matriz apresenta os termos trigonométricos expandidos, mas é matematicamente equivalente à matriz calculada manualmente após simplificação.

*This matrix has the expanded trigonometric terms, but is mathematically equivalent to the manually calculated matrix after simplification.*

# Simplificação do resultado / *Result simplification*:

- **T'(0,0):** $$\(-sin(\theta_1)sin(\theta_2)cos(\theta_0) + cos(\theta_0)cos(\theta_1)cos(\theta_2)\)$$  
  = $$\( cos(\theta_0) [cos(\theta_1)cos(\theta_2) - sin(\theta_1)sin(\theta_2)] \)$$  
  = $$\( cos(\theta_0)cos(\theta_1 + \theta_2) \)$$
  
- **T'(0,1):** $$\(-sin(\theta_1)cos(\theta_0)cos(\theta_2) - sin(\theta_2)cos(\theta_0)cos(\theta_1)\)$$  
  = $$\( -cos(\theta_0) [sin(\theta_1)cos(\theta_2) + cos(\theta_1)sin(\theta_2)] \)$$  
  = $$\( -cos(\theta_0)sin(\theta_1 + \theta_2) \)$$

- **T'(0,3):** $$\( a_1cos(\theta_0)cos(\theta_1) - a_2sin(\theta_1)sin(\theta_2)cos(\theta_0) + a_2cos(\theta_0)cos(\theta_1)cos(\theta_2) \)$$  
  = $$\( a_1cos(\theta_0)cos(\theta_1) + a_2cos(\theta_0) [cos(\theta_1)cos(\theta_2) - sin(\theta_1)sin(\theta_2)] \)$$  
  = $$\( a_1cos(\theta_0)cos(\theta_1) + a_2cos(\theta_0)cos(\theta_1 + \theta_2) \)$$

- **T'(1,0):** $$\(-sin(\theta_0)sin(\theta_1)sin(\theta_2) + sin(\theta_0)cos(\theta_1)cos(\theta_2)\)$$  
  = $$\( sin(\theta_0) [cos(\theta_1)cos(\theta_2) - sin(\theta_1)sin(\theta_2)] \)$$  
  = $$\( sin(\theta_0)cos(\theta_1 + \theta_2) \)$$

- **T'(1,1):** $$\(-sin(\theta_0)sin(\theta_1)cos(\theta_2) - sin(\theta_0)sin(\theta_2)cos(\theta_1)\)$$  
  = $$\( -sin(\theta_0) [sin(\theta_1)cos(\theta_2) + cos(\theta_1)sin(\theta_2)] \)$$  
  = $$\( -sin(\theta_0)sin(\theta_1 + \theta_2) \)$$

- **T'(1,3):** $$\( a_1sin(\theta_0)cos(\theta_1) - a_2sin(\theta_0)sin(\theta_1)sin(\theta_2) + a_2sin(\theta_0)cos(\theta_1)cos(\theta_2) \)$$  
  = $$\( a_1sin(\theta_0)cos(\theta_1) + a_2sin(\theta_0) [cos(\theta_1)cos(\theta_2) - sin(\theta_1)sin(\theta_2)] \)$$  
  = $$\( a_1sin(\theta_0)cos(\theta_1) + a_2sin(\theta_0)cos(\theta_1 + \theta_2) \)$$

- **T'(2,0):** $$\( sin(\theta_1)cos(\theta_2) + sin(\theta_2)cos(\theta_1) \)$$  
  = $$\( sin(\theta_1 + \theta_2) \)$$

- **T'(2,1):** $$\( -sin(\theta_1)sin(\theta_2) + cos(\theta_1)cos(\theta_2) \)$$  
  = $$\( cos(\theta_1)cos(\theta_2) - sin(\theta_1)sin(\theta_2) \)$$  
  = $$\( cos(\theta_1 + \theta_2) \)$$

- **T'(2,3):** $$\( a_1sin(\theta_1) + a_2sin(\theta_1)cos(\theta_2) + a_2sin(\theta_2)cos(\theta_1) \)$$  
  = $$\( a_1sin(\theta_1) + a_2 [sin(\theta_1)cos(\theta_2) + cos(\theta_1)sin(\theta_2)] \)$$  
  = $$\( a_1sin(\theta_1) + a_2sin(\theta_1 + \theta_2) \)$$



## **Trajetória calculada / *Calculated trajectory*:**

![image](https://github.com/user-attachments/assets/903c9679-f460-4f5c-b5fe-55307f81f451)


## **Parâmetros utilizados / *Used Parameters*:**

| Joint (i) | θᵢ (rad) | dᵢ (mm)  | aᵢ (mm) | αᵢ (rad) |
|-----------|---------:|--------:|-------:|---------:|
| 1         |   0.5    |    0   |   5    |    $\pi/2$     |
| 2         |   $\pi/4$    |    0   |   10    |    0     |
| 3         |   $\pi/6$     |    0   |   15    |    0     |




| Joint (i) | $\phi$ᵢ (rad) | $\epsilon$ᵢ (mm)  | $\sigma$ᵢ (mm) | αᵢ (rad) |
|-----------|---------:|--------:|-------:|---------:|
| 1         |   0.001    |    0.02   |   0.01    |    0.0003     |
| 2         |   0.005    |    0.04   |   0.05    |    0.0006     |
| 3         |   0.015    |    0.06   |   0.10    |    0.0009     |


### Referência usada / Reference used: 
[Robotics: Modelling, Planning and Control; Sciavicco, et al. 2009](https://link.springer.com/book/10.1007/978-1-84628-642-1)

* O acesso ao material é provido pela Universidade Federal de Santa Catarina (UFSC)
* *The access is provided by Federal University of Santa Catarina (UFSC)*
