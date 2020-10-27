%% Laboratorio de Proyectos II (EC3883)
% Profesor: José Cappelletto


%% Simulación: Motor DC con escobillas

% Grupo 2
% - Yuraily Bermejo (13-10142)
% - Vincenzo D'Argento (13-10334)
% - Jaime Villegas (13-11493)
% - Franz García (13-10505)

% Miembros encargados: - Jaime Villegas
%                      - Franz García


%% I. Parámetros del motor

% Leyenda: i -> Motor izquierdo (Rojo-Negro)
%          d -> Motor derechio (Verde-Blanco)


% 1. Resistencias de armadura (ohm)
Rd = 1.312;
Ri = 1.345;

% 2. Inductancias de Armadura (H)

% Constantes de tiempo (s)
Td = 130e-6;
Ti = 110e-6;

Ld = Td*(1+Rd);
Li = Ti*(1+Ri);


% 3. Constantes eléctricas (V*s)
Kd = 0.2506;
Ki = 0.2530;


% 4. Momentos de inercia

% a) Inercia del eje

% - Densidad del acero (Kg/m^3)
p_acero = 7850;

% - Volumen del eje (m^3)
r_eje = 1e-3;
h_eje = 38e-3;
V_eje = pi*h_eje*r_eje^2;

% - Masa del eje (Kg)
M_eje = V_eje*p_acero;

% - Momento de inercia (Kg*m^2)
J_eje = M_eje*r_eje^2 / 2;

% - Relación de la caja de engranajes
N = 203;

% - Inercia reflejada a través de la transmisión (Kg*m^2)
Je = N^2 * J_eje;

% b) Inercia de las ruedas 

% - Densidad del caucho (Kg/m^3)
p_caucho = 950;

% - Volumen de la rueda (m^3)
r1_rueda = 1.15e-2;
r2_rueda = 1.75e-2;
h_rueda = 1.5e-2;
V_rueda = pi*h_rueda*(r2_rueda^2 - r1_rueda^2);

% - Masa de la rueda (Kg)
M_rueda = V_rueda*p_caucho;

% - Momento de inercia (Kg*m^2)
Jr = M_rueda*(r1_rueda^2 + r2_rueda^2) / 2;

% c) Inercia total (Kg*m^2)
J = Je + Jr;

% 5. Constante de fricción viscosa

Bd = 0.01979;
Bi = 0.01890;



%% II. Respuesta al escalón (tras ejecutar simulación)

%Voltaje de armadura
Va = 1.0;

% Motor derecho
 subplot(2,1,1)
 plot(simout_d.Time, simout_d.Data, 'r')
 xlabel('tiempo (s)')
 ylabel('\omega (rad/s)')
 title(['Motor derecho (Rojo-Negro) - Va = ' num2str(Va) ' V' ])

% Motor izquierdo
 subplot(2,1,2)
 plot(simout_i.Time, simout_i.Data, 'g')
 xlabel('tiempo (s)')
 ylabel('\omega (rad/s)')
 title(['Motor izquierdo (Verde-Blanco) - Va = ' num2str(Va) ' V' ])
 

%% III. Funciones de transferencia
 
 % Motor derecho
Gp_d = tf(Kd,[Ld*J, Rd*J+Bd*Ld, Rd*Bd+Kd^2]);

% Motor izquierdo
Gp_i = tf(Ki,[Li*J, Ri*J+Bi*Li, Ri*Bi+Ki^2]);


