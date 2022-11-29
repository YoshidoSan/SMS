function MinimalWorkingExample()
    addpath('D:\SerialCommunication'); % add a path to the functions
    initSerialControl COM4 % initialise com port
    step_response=[];
    
    E=0;
    
    s_load = load("odp_skok_zoptymalizowana_duza.mat");
    s = s_load.y;
%     s_zak_load = load("odp_skok_zoptymalizowana_zak.mat");
%     s_zak = s_zak_load.y;
    
    % parametry dyskretnego regulatora DMC
    D = 450;
    N = 260;
    N_u = 260;
    lambda = 1;
    
    start = D+1;

    %warunki początkowe
    yzad(start-1:1:50+start)=33.7;
    yzad(51+start:1:250+start)=36;
    yzad(251+start:1:500+start)=39;
    U(1:500+start) = 27;
    e(start-1:1:start+600) = 0;
    
    
    %Obliczenie części macierzy DMC
    M = zeros(N, N_u);
    for column=1:N_u
        for row=1:N
            if row - column + 1 >= 1
                M(row, column) = s(row - column + 1);
            else
                M(row, column) = 0;
            end
        end
    end

    K = (M'*M+lambda*eye(N_u, N_u))^(-1)*M';

    M_p = zeros(N, D-1);
    for column=1:(D-1)
        for row=1:N
            if row + column > D
                M_p(row, column) = s(D) - s(D-1);
            else
                M_p(row, column) = s(row + column) - s(column);
            end
        end
    end
    
    %inicjalizacja pozostałych potrzebnych macierzy
    DU_p = zeros(D-1, 1);

    k = start;
    while(1)
        
        %% obtaining measurements
        measurements = readMeasurements(1); % read measurements from 1 to 1
        
        e(k)=yzad(k)-measurements;
        
        %Obliczenie DU_p
        for d=1:(D-1)
            DU_p(d) = U(k-d) - U(k-d-1);
        end

        %Pomiar wyjścia
        Y = ones(N, 1) * measurements;

        %Obliczenie Y_0
        yo = M_p * DU_p + Y;

        Y_zad = ones(N, 1) * yzad(k);

        %Obliczenie sterowania
        DU = K * (Y_zad - yo);
        U(k) = U(k-1) + DU(1);
        
        if U(k)>100
            U(k)=100;
        end
        if U(k)<0
            U(k)=0;
        end   
        
        %% processing of the measurements and new control values calculation
        disp("T: "+measurements+"; "+"U: "+U(k)+"; "+"Yzad: "+yzad(k));
        
        step_response=[step_response measurements(1)];
        
        save('DMC_data_lab4')
        %% sending new values of control signals
        % wentylator 1
        % gra�ka 5
        sendControls([ 1, 5], ... send for these elements
                     [ 50, U(k)]);  % new corresponding control valuesdisp(measurements); % process measurements
        
        %% synchronising with the control process
        waitForNewIteration(); % wait for new batch of measurements to be ready
        drawnow
 
        figure(1)
        plot(step_response)
        hold on
        plot(yzad(k))
        
        E = E + (yzad(k)-measurements)^2; 
        
        k = k +1;
    end
end