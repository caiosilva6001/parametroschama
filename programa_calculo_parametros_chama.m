%%% PROGRAMA PARA CÁLCULO DE RAZÃO DE EQUIVALÊNCIA, VAZÃO DA MISTURA E
%%% VELOCIDADE DE CHAMA PARA UM SLOT BURNER

close all; %fecha todas as figuras
clear all; %apaga todas as variaveis
clc; %limpa a janela de comandos

k=1; % definicao de variavel para repeticao de calculo
i=1;


while k<=7

    disp(" A ordem dos modelos deve ser: G3, G2, G1, M3, P3, LG-TEO, Q1 ");
    disp(['Para o modelo ' num2str(k) ' insira os parametros: ']);

    altura_modelo_mm(k)= input('Digite a altura (mm): ');
    espessura_fenda_mm(k)= input('Digite a espessura da fenda (mm): ');
    comprimento_fenda_mm(k)= input('Digite o comprimento da fenda (mm): ');
    A_fenda_m2(k)=(espessura_fenda_mm(k)*(10^(-3)))*(comprimento_fenda_mm(k)*(10^(-3)));
    a(k)=espessura_fenda_mm(k)*(10^(-3));
    b(k)=comprimento_fenda_mm(k)*(10^(-3));

    while i<=5
        
      delta_p_ar(i)= input('Digite a pressao do ar medida (kPa): ');
      delta_p_fuel(i)= input('Digite a pressao do combustivel medida (kPa): ');
      alfa_graus(i)= input('Digite o ângulo da chama com a direção paralela a vazao da mistura (graus): ');

      disp("\n Entradas \n\n");
      disp(['Altura_modelo: ' num2str(altura_modelo_mm(k)) ' ' 'mm']);
      disp(['Espessura_fenda: ' num2str(espessura_fenda_mm(k)) ' ' 'mm']);
      disp(['Comprimento_fenda: ' num2str(comprimento_fenda_mm(k)) ' ' 'mm']);
      disp(['Pressao_ar: ' num2str(delta_p_ar(i)) ' ' 'KPa']);
      disp(['Pressao_fuel: ' num2str(delta_p_fuel(i)) ' ' 'KPa']);
      disp(['Alfa: ' num2str(alfa_graus(i)) ' ' 'graus']);
    
      delta_p_ar_pa(i)=delta_p_ar(i)*(10^3);
      delta_p_fuel_pa(i)=delta_p_fuel(i)*(10^3);  
      razao_equivalencia(i)=0.24/(sqrt(delta_p_ar_pa(i)/delta_p_fuel_pa(i)));
      incerteza_razao_equivalencia(i)=sqrt(((0.0036*delta_p_fuel_pa(i))/(delta_p_ar_pa(i)^3))+(0.0036/(delta_p_ar_pa(i)*delta_p_fuel_pa(i))));
      alfa_rad(i)=deg2rad(alfa_graus(i));
      velocidade_queima(i)=(((19.66*(10^(-6))*(delta_p_ar_pa(i)^(1/2)))+(3.076*(10^(-7))*(delta_p_fuel_pa(i)^(1/2))))*sin(alfa_rad(i)))/(1.2301*A_fenda_m2(k));
      velocidade_queima_cm_per_s(i)=velocidade_queima(i)*100;
      incerteza_velocidade_queima_parcela_1=(1.59649*(10^-11)*((sin(alfa_rad(i)))^2))/(((A_fenda_m2(k))^2)*delta_p_ar_pa(i));
      incerteza_velocidade_queima_parcela_2=(3.90816*(10^-15)*((sin(alfa_rad(i)))^2))/(((A_fenda_m2(k))^2)*delta_p_fuel_pa(i));
      incerteza_velocidade_queima(i)=sqrt(incerteza_velocidade_queima_parcela_1+incerteza_velocidade_queima_parcela_2);
      incerteza_velocidade_queima_cm_per_s(i)=incerteza_velocidade_queima(i)*100;
      reynolds(i)=(0.126*a(k)*b(k)*((19.66*(delta_p_ar_pa(i)^(1/2)))+(0.3076*(delta_p_fuel_pa(i)^(1/2))))*sin(alfa_rad(i)))/((a(k)+b(k))*A_fenda_m2(k));
      incerteza_reynolds_parcela_1(i)=(0.38352*(a(k)^2)*(b(k)^2)*((sin(alfa_rad(i)))^2))/(((A_fenda_m2(k))^2)*delta_p_ar_pa(i)*((a(k)+b(k))^2));
      incerteza_reynolds_parcela_2(i)=(9.38845*(10^-5)*(a(k)^2)*(b(k)^2)*((sin(alfa_rad(i)))^2))/(((A_fenda_m2(k))^2)*delta_p_fuel_pa(i)*((a(k)+b(k))^2));
      incerteza_reynolds(i)=sqrt(incerteza_reynolds_parcela_1(i)+incerteza_reynolds_parcela_2(i));


        disp("\n Saidas \n\n");
        disp(['razao_equivalencia: ' num2str(razao_equivalencia(i)) '+/-' num2str(incerteza_razao_equivalencia(i))]);
        disp(['velocidade_de_chama: ' num2str(velocidade_queima_cm_per_s(i)) '+/-' num2str(incerteza_velocidade_queima_cm_per_s(i))  'cm/s']);
        disp(['reynolds: ' num2str(reynolds(i)) '+/-' num2str(incerteza_reynolds(i))]);
        
        i=i+1;

    end
    vetor_razao_equivalencia(k,1:5)=razao_equivalencia;
    vetor_incerteza_razao_equivalencia(k,1:5)=incerteza_razao_equivalencia;
    vetor_alfa(k,1:5)=alfa_graus;
    vetor_velocidade_chama(k,1:5)=velocidade_queima_cm_per_s;
    vetor_incerteza_velocidade_chama(k,1:5)=incerteza_velocidade_queima_cm_per_s;
    vetor_reynolds(k,1:5)=reynolds;
    vetor_incerteza_reynolds(k,1:5)=incerteza_reynolds;

    k=k+1;
    i=1;

    p=input('Digite 1  para calcular os parâmetros do proximo modelo ,  ou  digite  2 para parar:  ');
    if p==2
        break;
    end
    disp('Digite os parametros do proximo modelo: ');
end

% Definicao de modelos

%G3
G3_razao_equivalencia=vetor_razao_equivalencia(1,:);
G3_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(1,:);
G3_alfa=vetor_alfa(1,:);
G3_velocidade_chama=vetor_velocidade_chama(1,:);
G3_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(1,:);
G3_reynolds=vetor_reynolds(1,:);
G3_incerteza_reynolds=vetor_incerteza_reynolds(1,:);

%G2
G2_razao_equivalencia=vetor_razao_equivalencia(2,:);
G2_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(2,:);
G2_alfa=vetor_alfa(2,:);
G2_velocidade_chama=vetor_velocidade_chama(2,:);
G2_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(2,:);
G2_reynolds=vetor_reynolds(2,:);
G2_incerteza_reynolds=vetor_incerteza_reynolds(2,:);

%G1
G1_razao_equivalencia=vetor_razao_equivalencia(3,:);
G1_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(3,:);
G1_alfa=vetor_alfa(3,:);
G1_velocidade_chama=vetor_velocidade_chama(3,:);
G1_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(3,:);
G1_reynolds=vetor_reynolds(3,:);
G1_incerteza_reynolds=vetor_incerteza_reynolds(3,:);

%M3
M3_razao_equivalencia=vetor_razao_equivalencia(4,:);
M3_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(4,:);
M3_alfa=vetor_alfa(4,:);
M3_velocidade_chama=vetor_velocidade_chama(4,:);
M3_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(4,:);
M3_reynolds=vetor_reynolds(4,:);
M3_incerteza_reynolds=vetor_incerteza_reynolds(4,:);

%P3
P3_razao_equivalencia=vetor_razao_equivalencia(5,:);
P3_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(5,:);
P3_alfa=vetor_alfa(5,:);
P3_velocidade_chama=vetor_velocidade_chama(5,:);
P3_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(5,:);
P3_reynolds=vetor_reynolds(5,:);
P3_incerteza_reynolds=vetor_incerteza_reynolds(5,:);

%LG-TEO
LG_TEO_razao_equivalencia=vetor_razao_equivalencia(6,:);
LG_TEO_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(6,:);
LG_TEO_alfa=vetor_alfa(6,:);
LG_TEO_velocidade_chama=vetor_velocidade_chama(6,:);
LG_TEO_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(6,:);
LG_TEO_reynolds=vetor_reynolds(6,:);
LG_TEO_incerteza_reynolds=vetor_incerteza_reynolds(6,:);

%Q1
Q1_razao_equivalencia=vetor_razao_equivalencia(7,:);
Q1_incerteza_razao_equivalencia=vetor_incerteza_razao_equivalencia(7,:);
Q1_alfa=vetor_alfa(7,:);
Q1_velocidade_chama=vetor_velocidade_chama(7,:);
Q1_incerteza_velocidade_chama=vetor_incerteza_velocidade_chama(7,:);
Q1_reynolds=vetor_reynolds(7,:);
Q1_incerteza_reynolds=vetor_incerteza_reynolds(7,:);


%Aproximacao polinomial das funcoes velocidade de chama(razao de
%equivalencia)

p_G3=polyfit(G3_razao_equivalencia,G3_velocidade_chama,2);
G3tam=linspace(G3_razao_equivalencia(1,1),G3_razao_equivalencia(1,5),50);
G3_su_ajust=polyval(p_G3,G3tam);

p_G2=polyfit(G2_razao_equivalencia,G2_velocidade_chama,2);
G2tam=linspace(G2_razao_equivalencia(1,1),G2_razao_equivalencia(1,5),50);
G2_su_ajust=polyval(p_G2,G2tam);

p_G1=polyfit(G1_razao_equivalencia,G1_velocidade_chama,2);
G1tam=linspace(G1_razao_equivalencia(1,1),G1_razao_equivalencia(1,5),50);
G1_su_ajust=polyval(p_G1,G1tam);

p_M3=polyfit(M3_razao_equivalencia,M3_velocidade_chama,2);
M3tam=linspace(M3_razao_equivalencia(1,1),M3_razao_equivalencia(1,5),50);
M3_su_ajust=polyval(p_M3,M3tam);

p_P3=polyfit(P3_razao_equivalencia,P3_velocidade_chama,2);
P3tam=linspace(P3_razao_equivalencia(1,1),P3_razao_equivalencia(1,5),50);
P3_su_ajust=polyval(p_P3,P3tam);

p_LG_TEO=polyfit(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,2);
LG_TEOtam=linspace(LG_TEO_razao_equivalencia(1,1),LG_TEO_razao_equivalencia(1,5),50);
LG_TEO_su_ajust=polyval(p_LG_TEO,LG_TEOtam);

p_Q1=polyfit(Q1_razao_equivalencia,Q1_velocidade_chama,2);
Q1tam=linspace(Q1_razao_equivalencia(1,1),Q1_razao_equivalencia(1,5),50);
Q1_su_ajust=polyval(p_Q1,Q1tam);


%Grafico comparativo razao de equivalencia versus velocidade de chama
%(G3,G2,G1,M3,P3)
figure;
plot(G3tam,G3_su_ajust,'-'); %G3
hold on

plot(G2tam,G2_su_ajust,'-'); %G2
hold on

plot(G1tam,G1_su_ajust,'-'); %G1
hold on

plot(M3tam,M3_su_ajust,'-'); %M3
hold on

plot(P3tam,P3_su_ajust,'-'); %P3
legend('G3','G2','G1','M3','P3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Razao de Equivalencia versus Su com aproximacao polinomial')

hold off



%Graficos sem aproximacao polinomial
figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'x'); %G3
hold on

plot(G2_razao_equivalencia,G2_velocidade_chama,'x'); %G2
hold on

plot(G1_razao_equivalencia,G1_velocidade_chama,'x'); %G1
hold on

plot(M3_razao_equivalencia,M3_velocidade_chama,'x'); %M3
hold on

plot(P3_razao_equivalencia,P3_velocidade_chama,'x'); %P3
legend('G3','G2','G1','M3','P3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Razao de Equivalencia versus Su')

hold off

%Graficos isolados sem aproximacao polinomial
figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'x'); %G3
legend('G3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (G3)');
grid on

figure;
plot(G2_razao_equivalencia,G2_velocidade_chama,'x'); %G2
legend('G2');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (G2)');
grid on

figure;
plot(G1_razao_equivalencia,G1_velocidade_chama,'x'); %G1
legend('G1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (G1)');
grid on

figure;
plot(M3_razao_equivalencia,M3_velocidade_chama,'x'); %M3
legend('M3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (M3)');
grid on

figure;
plot(P3_razao_equivalencia,P3_velocidade_chama,'x'); %P3
legend('P3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (P3)');
grid on

figure;
plot(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,'x'); %LG_TEO
legend('LG-TEO');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (LG-TEO)');
grid on

figure;
plot(Q1_razao_equivalencia,Q1_velocidade_chama,'x'); %Q1
legend('Q1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su com aproximacao quadratica (Q1)');
grid on


%graficos sem aproximacao polinomial com erro
figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'x'); %G3
errorbar(G3_razao_equivalencia,G3_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_razao_equivalencia,G3_incerteza_razao_equivalencia,'.');
hold on

plot(G2_razao_equivalencia,G2_velocidade_chama,'x'); %G2
errorbar(G2_razao_equivalencia,G2_velocidade_chama,G2_incerteza_velocidade_chama,G2_incerteza_velocidade_chama,G2_incerteza_razao_equivalencia,G2_incerteza_razao_equivalencia,'.');
hold on

plot(G1_razao_equivalencia,G1_velocidade_chama,'x'); %G1
errorbar(G1_razao_equivalencia,G1_velocidade_chama,G1_incerteza_velocidade_chama,G1_incerteza_velocidade_chama,G1_incerteza_razao_equivalencia,G1_incerteza_razao_equivalencia,'.');
hold on

plot(M3_razao_equivalencia,M3_velocidade_chama,'x'); %M3
errorbar(M3_razao_equivalencia,M3_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_razao_equivalencia,M3_incerteza_razao_equivalencia,'.');
hold on

plot(P3_razao_equivalencia,P3_velocidade_chama,'x'); %P3
errorbar(P3_razao_equivalencia,P3_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_razao_equivalencia,P3_incerteza_razao_equivalencia,'.');
legend('G3','G2','G1','M3','P3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Razao de Equivalencia versus Su')

hold off


%Graficos isolados com erro
figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'.'); %G3
errorbar(G3_razao_equivalencia,G3_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_razao_equivalencia,G3_incerteza_razao_equivalencia,'.');
legend('G3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (G3)');
grid on

figure;
plot(G2_razao_equivalencia,G2_velocidade_chama,'.'); %G2
errorbar(G2_razao_equivalencia,G2_velocidade_chama,G2_incerteza_velocidade_chama,G2_incerteza_velocidade_chama,G2_incerteza_razao_equivalencia,G2_incerteza_razao_equivalencia,'.')
legend('G2');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (G2)');
grid on

figure;
plot(G1_razao_equivalencia,G1_velocidade_chama,'.'); %G1
errorbar(G1_razao_equivalencia,G1_velocidade_chama,G1_incerteza_velocidade_chama,G1_incerteza_velocidade_chama,G1_incerteza_razao_equivalencia,G1_incerteza_razao_equivalencia,'.')
legend('G1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (G1)');
grid on

figure;
plot(M3_razao_equivalencia,M3_velocidade_chama,'.'); %M3
errorbar(M3_razao_equivalencia,M3_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_razao_equivalencia,M3_incerteza_razao_equivalencia,'.')
legend('M3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (M3)');
grid on

figure;
plot(P3_razao_equivalencia,P3_velocidade_chama,'.'); %P3
errorbar(P3_razao_equivalencia,P3_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_razao_equivalencia,P3_incerteza_razao_equivalencia,'.')
legend('P3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (P3)');
grid on

figure;
plot(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,'.'); %LG_TEO
errorbar(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,LG_TEO_incerteza_velocidade_chama,LG_TEO_incerteza_velocidade_chama,LG_TEO_incerteza_razao_equivalencia,LG_TEO_incerteza_razao_equivalencia,'.')
legend('LG-TEO');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (LG-TEO)');
grid on

figure;
plot(Q1_razao_equivalencia,Q1_velocidade_chama,'.'); %Q1
errorbar(Q1_razao_equivalencia,Q1_velocidade_chama,Q1_incerteza_velocidade_chama,Q1_incerteza_velocidade_chama,Q1_incerteza_razao_equivalencia,Q1_incerteza_razao_equivalencia,'.')
legend('Q1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]');
title('Razao de Equivalencia versus Su (Q1)');
grid on

%Grafico Comparativo G3,LG-TEO,Q1 sem aproximacao polinomial e com erro
figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'x'); %G3
errorbar(G3_razao_equivalencia,G3_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_razao_equivalencia,G3_incerteza_razao_equivalencia,'.')
hold on

plot(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,'x'); %LG-TEO
errorbar(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,LG_TEO_incerteza_velocidade_chama,LG_TEO_incerteza_velocidade_chama,LG_TEO_incerteza_razao_equivalencia,LG_TEO_incerteza_razao_equivalencia,'.')
hold on

plot(Q1_razao_equivalencia,Q1_velocidade_chama,'x'); %Q1
errorbar(Q1_razao_equivalencia,Q1_velocidade_chama,Q1_incerteza_velocidade_chama,Q1_incerteza_velocidade_chama,Q1_incerteza_razao_equivalencia,Q1_incerteza_razao_equivalencia,'.')
legend('G3','LG-TEO','Q1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Su para diferentes modelos com Fenda 1x15 mm')

hold off

%Grafico Comparativo G3,LG-TEO,Q1 sem aproximacao polinomial e sem erro
figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'x'); %G3
hold on

plot(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,'x'); %LG-TEO
hold on

plot(Q1_razao_equivalencia,Q1_velocidade_chama,'x'); %Q1
legend('G3','LG-TEO','Q1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Su para diferentes modelos com Fenda 1x15 mm')

hold off

%Grafico Comparativo G3,LG-TEO,Q1 + Dirrenberger, P., et al. sem aproximacao polinomial e sem erro
% dados coletados de Dirrenberger, P., et al.
razao_equivalencia_bibliografia=[0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1.0 1.05 1.1 1.15 1.2 1.25 1.3 1.35 1.4 1.45 1.5 1.55 1.6 1.7 1.8 1.9 2];
velocidade_chama_bibliografia=[13.75 18 22.5 26.25 29.5 32.5 35.5 37.5 39.5 41 41 40 36 33 29 24.5 20 16.5 13 11 9 7.5 5 4 4];

figure;
plot(G3_razao_equivalencia,G3_velocidade_chama,'x'); %G3
hold on

plot(LG_TEO_razao_equivalencia,LG_TEO_velocidade_chama,'x'); %LG-TEO
hold on

plot(Q1_razao_equivalencia,Q1_velocidade_chama,'x'); %Q1
hold on

plot(razao_equivalencia_bibliografia,velocidade_chama_bibliografia,'x'); %Bibliografia
legend('G3','LG-TEO','Q1','Dirrenberger, P., et al.');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Su para diferentes modelos com Fenda 1x15 mm e Literatura')

hold off

%Grafico Comparativo G3,LG-TEO,Q1 + Dirrenberger, P., et al. sem aproximacao polinomial e sem erro
% dados coletados de Dirrenberger, P., et al.
razao_equivalencia_bibliografia=[0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1.0 1.05 1.1 1.15 1.2 1.25 1.3 1.35 1.4 1.45 1.5 1.55 1.6 1.7 1.8 1.9 2];
velocidade_chama_bibliografia=[13.75 18 22.5 26.25 29.5 32.5 35.5 37.5 39.5 41 41 40 36 33 29 24.5 20 16.5 13 11 9 7.5 5 4 4];

figure;
plot(G3tam,G3_su_ajust,'-'); %G3
hold on

plot(LG_TEOtam,LG_TEO_su_ajust,'-'); %LG-TEO
hold on

plot(Q1tam,Q1_su_ajust,'-'); %Q1
hold on

plot(razao_equivalencia_bibliografia,velocidade_chama_bibliografia,'x'); %Bibliografia
legend('G3','LG-TEO','Q1','Dirrenberger, P., et al.');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Aproximacao Su para diferentes modelos com Fenda 1x15 mm e Literatura')

hold off

%Grafico Comparativo G3,LG-TEO,Q1 com aproximacao polinomial sem erro
figure;
plot(G3tam,G3_su_ajust,'-'); %G3
hold on

plot(LG_TEOtam,LG_TEO_su_ajust,'-'); %LG-TEO
hold on

plot(Q1tam,Q1_su_ajust,'-'); %Q1
legend('G3','LG-TEO','Q1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Su com Aproximação Polinomial para diferentes modelos')

hold off


%Grafico Comparativo G3,M3,P3 com aproximacao polinomial sem erro
figure;
plot(G3tam,G3_su_ajust,'-'); %G3
hold on

plot(M3tam,M3_su_ajust,'-'); %M3
hold on

plot(P3tam,P3_su_ajust,'-'); %P3
legend('G3','M3','P3');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Su com Aproximação Polinomial para G3,M3 e P3')

hold off

%Grafico Comparativo G3,G2,G1 com aproximacao polinomial sem erro
figure;
plot(G3tam,G3_su_ajust,'-'); %G3
hold on

plot(G2tam,G2_su_ajust,'-'); %G2
hold on

plot(G1tam,G1_su_ajust,'-'); %G1
legend('G3','G2','G1');
xlabel('razao de equivalencia');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Razao de Equivalencia versus Su com Aproximação Polinomial para G3,G2 e G1')

hold off

%Grafico (Re vs Su) Comparativo G3,M3,P3 com linha de tendencia
figure;
plot(G3_reynolds,G3_velocidade_chama,'-'); %G3
hold on

plot(M3_reynolds,M3_velocidade_chama,'-'); %G3
hold on

plot(P3_reynolds,P3_velocidade_chama,'-'); %P3
legend('G3 (108 mm)','M3 (95 mm)','P3(72 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Numero de Reynolds versus Su com Aproximação Polinomial para G3, M3 e P3')

hold off

%Grafico Comparativo do Número de Reynolds G3,M3,P3 sem aproximacao
%polinomial e com erro (Reynolds versus Velocidade de Chama)

figure;
plot(G3_reynolds,G3_velocidade_chama,'x'); %G3
errorbar(G3_reynolds,G3_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_reynolds,G3_incerteza_reynolds,'.');
hold on

plot(M3_reynolds,M3_velocidade_chama,'x'); %M3
errorbar(M3_reynolds,M3_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_reynolds,M3_incerteza_reynolds,'.');
hold on

plot(P3_reynolds,P3_velocidade_chama,'x'); %P3
errorbar(P3_reynolds,P3_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_reynolds,P3_incerteza_reynolds,'.');
legend('G3 (108 mm)','M3 (95 mm)','P3(72 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Reynolds versus Su para diferentes alturas de modelo com Fenda 1x15mm')

hold off

%Grafico Comparativo do Número de Reynolds G3,M3,P3 sem aproximacao
%polinomial e com erro (Reynolds versus Velocidade de Chama)

figure;
plot(G3_reynolds,G3_velocidade_chama,'.'); %G3
errorbar(G3_reynolds,G3_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_reynolds,G3_incerteza_reynolds,'.');
hold on

plot(M3_reynolds,M3_velocidade_chama,'.'); %M3
errorbar(M3_reynolds,M3_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_reynolds,M3_incerteza_reynolds,'.');
hold on

plot(P3_reynolds,P3_velocidade_chama,'.'); %P3
errorbar(P3_reynolds,P3_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_reynolds,P3_incerteza_reynolds,'.');
legend('G3 (108 mm)','M3 (95 mm)','P3(72 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Reynolds versus Su para diferentes alturas de modelo com Fenda 1x15mm')

hold off

%Grafico Comparativo do Número de Reynolds G3,M3,P3 separados

figure;
plot(G3_reynolds,G3_velocidade_chama,'.'); %G3
errorbar(G3_reynolds,G3_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_velocidade_chama,G3_incerteza_reynolds,G3_incerteza_reynolds,'.');
legend('G3 (108 mm)','M3 (95 mm)','P3(72 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Reynolds versus Su - modelo G3 (108 mm de altura)')

figure;
plot(M3_reynolds,M3_velocidade_chama,'.'); %M3
errorbar(M3_reynolds,M3_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_velocidade_chama,M3_incerteza_reynolds,M3_incerteza_reynolds,'.');
legend('M3 (95 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Reynolds versus Su - modelo M3 (95 mm de altura)')

figure;
plot(P3_reynolds,P3_velocidade_chama,'.'); %P3
errorbar(P3_reynolds,P3_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_velocidade_chama,P3_incerteza_reynolds,P3_incerteza_reynolds,'.');
legend('P3(72 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Reynolds versus Su - modelo P3 (72 mm de altura)')

%Grafico Comparativo do Número de Reynolds G3,M3,P3 sem aproximacao
%polinomial e sem erro (Reynolds versus Velocidade de Chama)

figure;
plot(G3_reynolds,G3_velocidade_chama,'x'); %G3
hold on

plot(M3_reynolds,M3_velocidade_chama,'x'); %M3
hold on

plot(P3_reynolds,P3_velocidade_chama,'x'); %P3
legend('G3 (108 mm)','M3 (95 mm)','P3(72 mm)');
xlabel('Número de Reynolds');
ylabel('Velocidade de chama[cm/s]')
grid on
title('Comparação Reynolds versus Su para diferentes alturas de modelo com Fenda 1x15mm')

%hold off
