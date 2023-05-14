

%% Equations for lengths of the actuators
syms th1 th2 th3 real

R1=RotM(th1,1);
R2=RotM(th2,2);
R3=RotM(th3,3);
R = R1*R2*R3;

bottomPosVectors = [-432.625,-331.18,54.61;
                     432.625,-331.18,54.61;
					 503.125,-209.07,54.61;
					  70.500, 540.25,54.61;
					 -70.500, 540.25,54.61;
					-503.125,-209.07,54.61]';
                
topPosVectors = [ -40.00,-144.62,-50;
				   40.00,-144.62,-50;
				  145.22,  37.67,-50;
				  105.25, 106.95,-50;
				 -105.25, 106.95,-50;
				 -145.22,  37.67,-50]';

L = sym('L%d',[6,1],'real');
X = sym('x%d',[3,1],'real');
freedoms = [X',th1,th2,th3]';
equations = sym(zeros(6,1));

for i=1:6
    equations(i) = simplify(expand(sqrt(dot((X-bottomPosVectors(:,i))+R*topPosVectors(:,i), (X-bottomPosVectors(:,i))+R*topPosVectors(:,i)))-(L(i))));
end

NM = sym(zeros(6,6));

for i=1:6
    for j=1:6
        NM(i,j) = simplify(expand(diff(equations(i),freedoms(j))));
    end
end
%%







fidm = fopen('getNMandInvNM.m','w');

fprintf(fidm,'function [NM] = getNMandInvNM(x, th ) \n\n');

for i=1:3
fprintf(fidm,strcat("th",num2str(i)," = th(",num2str(i),");\n"));
end
for i=1:3
fprintf(fidm,strcat("x",num2str(i)," = x(",num2str(i),");\n"));
end

fprintf(fidm, "NM = zeros(6,6);\n\n");


fidc = fopen('getNewtonRapson.cpp','w');

fprintf(fidc,'#include <math.h> \n\n');

fprintf(fidc,'int getNewtonRapsonMatrix(double freedoms[6], double NewtonRapsonMatrix[6][6]){ \n\n');

for i=1:3

    fprintf(fidc, strcat('double th',num2str(i),' = freedoms[',num2str(i+2),'];\n')); 
end
for i=1:3

    fprintf(fidc, strcat('double x',num2str(i),' = freedoms[',num2str(i-1),'];\n')); 
end


%% NewtonRaphson matrix


for i= 1:6
    for j= 1:6
        fprintf(fidm, strcat('NM(',num2str(i),',',num2str(j),') = ',string(NM(i,j)),';\n\n')); %Matlab
        fprintf(fidc, strcat('NewtonRapsonMatrix[',num2str(i-1),'][',num2str(j-1),'] = ',string(NM(i,j)),';\n\n')); %cpp
    end 
end
fprintf(fidc,'\n\n return 0;\n\n}');
fclose(fidc);
fclose(fidm);
%%
fidL = fopen('getL.m','w');

fprintf(fidL,'function [L] = getL(x, th, length )  \n\n');

for i=1:3
fprintf(fidL,strcat("th",num2str(i)," = th(",num2str(i),");\n"));
end
for i=1:3
fprintf(fidL,strcat("x",num2str(i)," = x(",num2str(i),");\n"));
end

for i=1:6
fprintf(fidL,strcat("L",num2str(i)," = length(",num2str(i),");\n"));
end

fprintf(fidL, "L = zeros(6,1);\n\n");
for j=1:6
for i=1:6
    myString = string(equations(j));
    newString = strrep(myString, strcat("x",num2str(i),"^2"),strcat("x",num2str(i),"*","x",num2str(i)));
    newNewString = strrep(newString, strcat("L",num2str(i),"^2"),strcat("L",num2str(i),"*","L",num2str(i)));
end
fprintf(fidL,strcat("L(",num2str(j),") = ",newNewString,";\n"));
end

fprintf(fidL, "\n\n end\n\n");
fclose(fidL);
%% get L cpp
fidL = fopen('getL.cpp','w');
fprintf(fidL,'#include <math.h>  \n\n');

fprintf(fidL,'int  getLengthErrors(double lengthErrors[6], double freedoms[6], double actuatorLengths[6] ) {  \n\n');

for i=1:3
fprintf(fidL,strcat("double x",num2str(i)," = freedoms[",num2str(i-1),"];\n"));
end

for i=1:3
fprintf(fidL,strcat("double th",num2str(i)," = freedoms[",num2str(i+2),"];\n"));
end


for i=1:6
fprintf(fidL,strcat("double L",num2str(i)," = actuatorLengths[",num2str(i-1),"];\n"));
end

for j=1:6
    myString = string(equations(j));
for i=1:6
    myString = strrep(myString, strcat("x",num2str(i),"^2"),strcat("x",num2str(i),"*","x",num2str(i)));
    myString = strrep(myString, strcat("L",num2str(i),"^2"),strcat("L",num2str(i),"*","L",num2str(i)));
end
fprintf(fidL,strcat("lengthErrors[",num2str(j-1),"] = ",myString,";\n\n"));
end

fprintf(fidL,'\n\n return 0;\n\n}');
fclose(fidL);
