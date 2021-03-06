function T = WidowXFK()
    %Esta funci�n regresa la matriz de transformaci�n homog�nea simb�lica
    %desde la base del brazo hasta el centro de la herramienta.
    syms q1 q2 q3 q4 q5 L0  L1 L2 L3 L4 a D
    piSym = sym(pi);
    
    Tb0 = [eye(3) [0;0;L0]; 0 0 0 1]; %Matriz TH base --> origen
    T01 = MaTranH(q1,0,0,0); %Matriz TH origen --> 1
    T12 = MaTranH(q2+a, 0, piSym/2,0); %Matriz TH 1 --> 2
    T23 = MaTranH(q3-a,0,0,D);%Matriz TH 2 --> 3
    T34 = MaTranH(q4-piSym/2,0,0,L3); %Matriz TH 3 --> 4
    T45 = MaTranH(q5,0,-piSym/2,0); %Matriz TH 4 --> 5
    T5t = [RotY(-piSym/2) [0;0; L4]; 0 0 0 1]; %Matriz TH 5 --> tool
    T15 = simplify(T12*T23*T34)*T45; %Matriz TH 1 --> 5
    T1t = simplify(T15*T5t); %Matriz TH 1 --> tool
    T = simplify(Tb0*T01)*T1t; %Matriz TH base --> tool
    
end

function T = MaTranH(qi,di,alfai_1,ai_1)
    %Esta funci�n regresa la matriz de transformaci�n homog�nea
    %Utilizando los par�metros de Denavit y Hartenberg
    %Usa los par�metros MaTranH(qi,di,alfai_1,ai_1) (en radianes los �ngulos)
    %La matriz se calcula as�:
    %T = [cos(qi) -sin(qi) 0 ai_1;
    %  sin(qi)*cos(alfai_1) cos(qi)*cos(alfai_1) -sin(alfai_1) -di*sin(alfai_1);
    %  sin(qi)*sin(alfai_1) cos(qi)*sin(alfai_1) cos(alfai_1) di*cos(alfai_1);
    %  0 0 0 1];

    T = [cos(qi) -sin(qi) 0 ai_1;
      sin(qi)*cos(alfai_1) cos(qi)*cos(alfai_1) -sin(alfai_1) -di*sin(alfai_1);
      sin(qi)*sin(alfai_1) cos(qi)*sin(alfai_1) cos(alfai_1) di*cos(alfai_1);
      0 0 0 1
    ];

    if ~isnumeric([qi di alfai_1 ai_1])
        T = simplify(T);
    end
end
function Ry = RotY(q)
    Ry = [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
end
