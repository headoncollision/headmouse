%a = arduino('COM10', 'Uno', 'Libraries', 'I2C');
%fs = 100; % Sample Rate in Hz   
%imu = mpu9250(a,'SampleRate',fs);

% GyroscopeNoise and AccelerometerNoise is determined from datasheet.
%GyroscopeNoiseMPU9250 = 3.0462e-06; % GyroscopeNoise (variance value) in units of rad/s
%AccelerometerNoiseMPU9250 = 0.0061; % AccelerometerNoise(variance value)in units of m/s^2
%FUSE = ahrsfilter('SampleRate',imu.SampleRate, 'GyroscopeNoise',GyroscopeNoiseMPU9250,'AccelerometerNoise',AccelerometerNoiseMPU9250);

import java.awt.Robot;
mouse = Robot;
mouse.mouseMove(683,384);

counter = 0;
get_ort = [];

while counter < 100
      counter = counter + 1;
      
      % Home-Base
      imu_read = read(imu);
      imu_matrix = imu_read{:,:};
      imu_mean = mean(imu_matrix);
      
      % Accelerometer
      acc_x = imu_mean(:,1);
      acc_y = imu_mean(:,2);
      acc_z = imu_mean(:,3);
      accel_x = imu_matrix(:,1);
      accel_y = imu_matrix(:,2);
      accel_z = imu_matrix(:,3);
      accel_readings = [accel_x,accel_y,accel_z];
      accel_read = [acc_x,acc_y,acc_z];
      
      % Gyroscope
      gyro_x = imu_mean(:,4);
      gyro_y = imu_mean(:,5);
      gyro_z = imu_mean(:,6);
      gyroscope_x = imu_matrix(:,4);
      gyroscope_y = imu_matrix(:,5);
      gyroscope_z = imu_matrix(:,6);
      gyroscope_readings = [gyroscope_x,gyroscope_y,gyroscope_z];
      gyro_read = [gyro_x,gyro_y,gyro_z];
      
      % Magnetometer
      mag_x = imu_mean(:,7);
      mag_y = imu_mean(:,8);
      mag_z = imu_mean(:,9);
      magnet_x = imu_matrix(:,7);
      magnet_y = imu_matrix(:,8);
      magnet_z = imu_matrix(:,9);
      mag_readings = [magnet_x,magnet_y,magnet_z];
      mag_read = [mag_x,mag_y,mag_z];
      
      % Orientation
      [orientation,angularVelocity] = FUSE(accel_read,gyro_read,mag_read);
      [ort,av] = FUSE(accel_readings,gyroscope_readings,mag_readings);
      x = quat2rotm(ort);
      z = av*180/3.141926;
      orientation_table = array2table(ort);
      av_table = array2table(z);
      
      orient = quat2rotm(orientation);
      ort_rad = mean(orient);
      ort_deg = ((ort_rad*180)/3.1415926);
      
      ort_deg_x = (-250)*ort_deg(:,1);
      ort_deg_y = -(25)*ort_deg(:,2);
      ort_deg_z = (-200)*ort_deg(:,3);
      ort_deg_xyz = [ort_deg_x,ort_deg_y,ort_deg_z];
      
      angularVelocity_x = ((angularVelocity(:,1)*180)/3.1415926);
      angularVelocity_y = ((angularVelocity(:,2)*180)/3.1415926);
      angularVelocity_z = ((angularVelocity(:,3)*180)/3.1415926);
      ang_vel = [angularVelocity_x,angularVelocity_y,angularVelocity_z];
      angular_velocity_x = 683-(((ang_vel(:,1)^2)+(ang_vel(:,1))));
      angular_velocity_y = 384-(((ang_vel(:,2)^2)+(ang_vel(:,2))));
     
      mouse.mouseMove(angular_velocity_x, angular_velocity_y);
      
      %[angular_velocity_x,angular_velocity_y];
      %ang_vel
      %ort_deg
      %ort_deg_xyz
      
            
      %if angular_velocity_x < 0 
      %   angular_velocity_x = -1*angular_velocity_x;
      %   if  angular_velocity_y < 0
      %   angular_velocity_y = -1*angular_velocity_y;
      %   end
      %   mouse.mouseMove(angular_velocity_x, angular_velocity_y);
         
      %else
       %    mouse.mouseMove(angularVelocity_x,angularVelocity_y);
           
      %end     
      %ort_radians = det(orient);
      %ort_degrees = ((ort_radians*180)/3.1415926);
      %ort_degrees      
      
      if counter == 100
          disp('End of Session')
      end    
end   
%ort_radians = det(orient);
%ort_degrees = ((ort_radians*180)/3.1415926);
%ort_degrees
av_table      
