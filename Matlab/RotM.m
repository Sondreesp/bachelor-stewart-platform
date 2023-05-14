function [R] = RotM(Angle,Axis)

switch Axis
    
    case 1
        
        R=[ 1,         0,          0;
            0,cos(Angle),-sin(Angle);
            0,sin(Angle),cos(Angle)];
        
    case 2
        
         R=[ cos(Angle),0,sin(Angle);
                      0,1,         0;
            -sin(Angle),0,cos(Angle)];
        
    case 3
        
         R=[cos(Angle),-sin(Angle),0;
            sin(Angle), cos(Angle),0;
                     0,          0,1];
                 
    otherwise
        error("Error in RotM, Axis is not 1, 2 or 3");
end

