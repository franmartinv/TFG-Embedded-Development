%% Script for read all data sensors and use it for see in graphics

%% First we save the .csv sensors data in some new variables
BME680_t        =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\BME680_Temperature.csv');
BME680_p        =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\BME680_Pressure.csv');
BME680_h        =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\BME680_Humidity.csv');
CCS811_TVOC     =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\CCS811_TVOC.csv');
CCS811_eco2     =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\CCS811_eco2.csv');
MS5611_t        =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\MS5611_Temperature.csv');
MS5611_p        =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\MS5611_Pressure.csv');

year            =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\Time\year.csv');
month           =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\Time\month.csv');
day             =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\Time\day.csv');
hour            =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\Time\hour.csv');
minute          =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\Time\minute.csv');
second          =   readmatrix('C:\Users\fmart\Desktop\Datos sensores TFG\Time\second.csv');

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
header = ["Year" "Month" "Day" "Hour" "Minute" "Second" "MS5611 Temperature" "MS5611 Pressure" "CCS811 TVOC" "CCS811 eco2" "BME680 Temperature" "BME680 Pressure" "BME680 Humidity"];
data = [t.Year, t.Month, t.Day, t.Hour, t.Minute, t.Second, MS5611_t, MS5611_p, CCS811_TVOC, CCS811_eco2, BME680_t, BME680_p, BME680_h];
table = [header;data];
filename = 'TFG_sensor_data_FRAN.xlsx';
writematrix(table, filename, 'Sheet', 1);

%% Graphics of sensor's data

figure
plot(BME680_h)
xlabel('Time')
ylabel('% RH')
title('[%RH] Humidity by BME680')
%legend('Humidity')
grid

figure
plot(t, BME680_p, "DatetimeTickFormat", "hh-mm-ss")
xlabel('Time')
ylabel('Pressure [mBar]')
title('Pressure in [mBar] by BME680')
%legend('Pressure')
grid

figure
plot(BME680_t)
xlabel('Time')
ylabel('Temperature [ºC]')
title('Temperature in [Celsius] by BME680')
%legend('Temperature')
grid

figure
plot(CCS811_TVOC)
xlabel('Time')
ylabel('Volatile organic compounds [ppm]')
title('TVOC in [ppm] by CCS811')
%legend('TVOC')
grid

figure
plot(CCS811_eco2)
xlabel('Time')
ylabel('CO2 concentration [ppm]')
title('CO2 concentration in [ppm] by CCS811')
%legend('eco2')
grid

figure
plot(MS5611_p)
xlabel('Time')
ylabel('Pressure [mBar]')
title('Pressure in [mBar] by MS5611')
%legend('Pressure')
grid

figure
plot(MS5611_t)
xlabel('Time')
ylabel('Temperature [ºC]')
title('Temperature in [Celsius] by MS5611')
%legend('Temperature')
grid


%% Sensor data in only one graphic
figure
subplot(2,4,1)
plot(MS5611_t)
xlabel('Time')
ylabel('Temperature [ºC]')
title('Temperature in [Celsius] by MS5611')
%legend('Temperature')
grid

subplot(2,4,2)
plot(MS5611_p)
xlabel('Time')
ylabel('Pressure [mBar]')
title('Pressure in [mBar] by MS5611')
%legend('Pressure')
grid

subplot(2,4,3)
plot(BME680_h)
xlabel('Time')
ylabel('% RH')
title('[%RH] Humidity by BME680')
%legend('Humidity')
grid

subplot(2,4,4)
plot(CCS811_TVOC)
xlabel('Time')
ylabel('Volatile organic compounds [ppm]')
title('TVOC in [ppm] by CCS811')
%legend('TVOC')
grid

subplot(2,4,5)
plot(BME680_t)
xlabel('Time')
ylabel('Temperature [ºC]')
title('Temperature in [Celsius] by BME680')
%legend('Temperature')
grid


subplot(2,4,6)
plot(BME680_p)
xlabel('Time')
ylabel('Pressure [mBar]')
title('Pressure in [mBar] by BME680')
%legend('Pressure')
grid

%subplot(2,4,7)

subplot(2,4,8)
plot(CCS811_eco2)
xlabel('Time')
ylabel('CO2 concentration [ppm]')
title('CO2 concentration in [ppm] by CCS811')
%legend('eco2')
grid
