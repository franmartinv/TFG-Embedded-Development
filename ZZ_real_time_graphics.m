%% Script for read all data sensors and use it for see in graphics

%% First we save the .csv sensors data in some new variables

Tm = 10;        % sample time of 10 seconds
i = 0;
figure

while(i<1)
    tic
    
    BME680_t        =   readmatrix('C:\tfg-sensors-data\BME680_Temperature.csv');
    BME680_p        =   readmatrix('C:\tfg-sensors-data\BME680_Pressure.csv');
    BME680_h        =   readmatrix('C:\tfg-sensors-data\BME680_Humidity.csv');
    CCS811_TVOC     =   readmatrix('C:\tfg-sensors-data\CCS811_TVOC.csv');
    CCS811_eco2     =   readmatrix('C:\tfg-sensors-data\CCS811_eco2.csv');
    MS5611_t        =   readmatrix('C:\tfg-sensors-data\MS5611_Temperature.csv');
    MS5611_p        =   readmatrix('C:\tfg-sensors-data\MS5611_Pressure.csv');

    year            =   readmatrix('C:\tfg-sensors-data\Time\year.csv');
    month           =   readmatrix('C:\tfg-sensors-data\Time\month.csv');
    day             =   readmatrix('C:\tfg-sensors-data\Time\day.csv');
    hour            =   readmatrix('C:\tfg-sensors-data\Time\hour.csv');
    minute          =   readmatrix('C:\tfg-sensors-data\Time\minute.csv');
    second          =   readmatrix('C:\tfg-sensors-data\Time\second.csv');

    %% Cutting and comparing vectors length
    len(1)          =   length(BME680_h);
    len(2)          =   length(BME680_p);
    len(3)          =   length(BME680_t);
    len(4)          =   length(CCS811_eco2);
    len(5)          =   length(CCS811_TVOC);
    len(6)          =   length(MS5611_p);
    len(7)          =   length(MS5611_t);
    len(8)          =   length(year);
    len(9)          =   length(month);
    len(10)         =   length(day);
    len(11)         =   length(hour);
    len(12)         =   length(minute);
    len(13)         =   length(second);

    n = min(len);

    BME680_h        =   BME680_h(1:n);
    BME680_p        =   BME680_p(1:n);
    BME680_t        =   BME680_t(1:n);
    CCS811_eco2     =   CCS811_eco2(1:n);
    CCS811_TVOC     =   CCS811_TVOC(1:n);
    MS5611_p        =   MS5611_p(1:n);
    MS5611_t        =   MS5611_t(1:n);
    year            =   year(1:n);
    month           =   month(1:n);
    day             =   day(1:n);
    hour            =   hour(1:n);
    minute          =   minute(1:n);
    second          =   second(1:n);

    % With this command I can create an array where I save all data
    % time configured with the following struct: 
    % day-month-year hour-minute-second 

    t = datetime(year, month, day, hour, minute, second);

    %% Export to Excel
    
    header = ["Year" "Month" "Day" "Hour" "Minute" "Second" "MS5611 Temperature [�C]" "MS5611 Pressure [mBar]" "CCS811 TVOC [ppb]" "CCS811 eco2 [ppm]" "BME680 Temperature [�C]" "BME680 Pressure [hPa]" "BME680 Humidity [%]"];
    data = [t.Year, t.Month, t.Day, t.Hour, t.Minute, t.Second, MS5611_t, MS5611_p, CCS811_TVOC, CCS811_eco2, BME680_t, BME680_p, BME680_h];
    table = [header;data];
    filename = 'TFG_sensor_data_FRAN.xlsx';
    writematrix(table, filename, 'Sheet', 1);

    %% Graphics of sensor's data

    % figure
    % plot(t,BME680_h)
    % xlabel('Time')
    % ylabel('% RH')
    % title('[%RH] Humidity by BME680')
    % %legend('Humidity')
    % grid
    % 
    % figure
    % plot(t, BME680_p)
    % xlabel('Time')
    % ylabel('Pressure [mBar]')
    % title('Pressure in [mBar] by BME680')
    % %legend('Pressure')
    % grid
    % 
    % figure
    % plot(t,BME680_t)
    % xlabel('Time')
    % ylabel('Temperature [�C]')
    % title('Temperature in [Celsius] by BME680')
    % %legend('Temperature')
    % grid
    % 
    % figure
    % plot(t,CCS811_TVOC)
    % xlabel('Time')
    % ylabel('Volatile organic compounds [ppm]')
    % title('TVOC in [ppb] by CCS811')
    % %legend('TVOC')
    % grid
    % 
    % figure
    % plot(t,CCS811_eco2)
    % xlabel('Time')
    % ylabel('CO2 concentration [ppm]')
    % title('CO2 concentration in [ppm] by CCS811')
    % %legend('eco2')
    % grid
    % 
    % figure
    % plot(t,MS5611_p)
    % xlabel('Time')
    % ylabel('Pressure [mBar]')
    % title('Pressure in [mBar] by MS5611')
    % %legend('Pressure')
    % grid
    % 
    % figure
    % plot(t,MS5611_t)
    % xlabel('Time')
    % ylabel('Temperature [�C]')
    % title('Temperature in [Celsius] by MS5611')
    % %legend('Temperature')
    % grid


%% Sensor data in only one graphic

    subplot(2,4,1)
    plot(t,MS5611_t)
    xlabel('Time')
    ylabel('Temperature [�C]')
    title('Temperature in [Celsius] by MS5611')
    %legend('Temperature')
    grid

    subplot(2,4,2)
    plot(t,MS5611_p)
    xlabel('Time')
    ylabel('Pressure [mBar]')
    title('Pressure in [mBar] by MS5611')
    %legend('Pressure')
    grid

    subplot(2,4,3)
    plot(t,BME680_h)
    xlabel('Time')
    ylabel('% RH')
    title('[%RH] Humidity by BME680')
    %legend('Humidity')
    grid

    subplot(2,4,4)
    plot(t,CCS811_TVOC)
    xlabel('Time')
    ylabel('Volatile organic compounds [ppb]')
    title('TVOC in [ppb] by CCS811')
    %legend('TVOC')
    grid

    subplot(2,4,5)
    plot(t,BME680_t)
    xlabel('Time')
    ylabel('Temperature [�C]')
    title('Temperature in [Celsius] by BME680')
    %legend('Temperature')
    grid


    subplot(2,4,6)
    plot(t,BME680_p)
    xlabel('Time')
    ylabel('Pressure [mBar]')
    title('Pressure in [mBar] by BME680')
    %legend('Pressure')
    grid

    %subplot(2,4,7)

    subplot(2,4,8)
    plot(t,CCS811_eco2)
    xlabel('Time')
    ylabel('CO2 concentration [ppm]')
    title('CO2 concentration in [ppm] by CCS811')
    %legend('eco2')
    grid
    
    sample_time = toc;
    
    pause(max(Tm,sample_time));
end
