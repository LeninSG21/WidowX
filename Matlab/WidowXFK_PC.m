function WidowXFK_PC(q0)
    %Esta función recibe un vector de tamaño 5, donde cada elemento
    %corresponde al ángulo de cada valor articular del brazo WidowX. A
    %partir de estos, dibuja el brazo manipulador usando la toolbox de
    %Peter Corke
    [r,c] = size(q0);
    if(c~=5 || r~=1)
        if(c~=1 || r~=5)
            disp('Dimensiones inválidas! Se espera un vector de dimensión 5')
            return
        end
        q0 = q0';
    end
    L0 = 9; L1 = 14; L2 = 5; L3 = 14; L4 = 14;
    a = atan(L1/L2); D = sqrt(L1^2+L2^2); 
    L(1) = Link([0 0 0 0 0 0], 'modified');
    L(2) = Link([0 0 0 pi/2 0 a], 'modified');
    L(3) = Link([0 0 D 0 0 -a], 'modified');
    L(4) = Link([0 0 L3 0 0 -pi/2], 'modified');
    L(5) = Link([0 0 0 -pi/2 0 0], 'modified');
    qlimit = pi*[-2 2; -0.5 0.5; -0.5 5/6; -11/18 0.5; -5/6 5/6];
    WidowX = SerialLink(L, 'name', 'Widow X II', 'qlim', qlimit);
    WidowX.tool = [RotY(-pi/2) [0;0;L4]; 0 0 0 1];
    WidowX.base = [eye(3) [0;0;L0]; 0 0 0 1];
    WidowX.plot(q0, 'workspace',2*[-40 40 -40 40 0 40]);
    WidowX.plot(q0, 'floorlevel',0);    
    WidowX.teach(q0);
    hold on;
    trplot(eye(4), 'frame','0','color','k');
    hold off;
end

function Ry = RotY(q)
    Ry = [cos(q) 0 sin(q); 0 1 0; -sin(q) 0 cos(q)];
end

