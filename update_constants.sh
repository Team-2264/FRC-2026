#!/bin/bash
sed -i '' 's/public static final double DefaultShooterDutyCycle = -0.6;/public static final double DefaultShooterRPM = -3360.0;/g' src/main/java/frc/robot/Constants.java

sed -i '' 's/speedTable.put(2.0\*0.3048,    -0.41);/speedTable.put(2.0*0.3048,    -2296.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(3.0\*0.3048,   -0.44);/speedTable.put(3.0*0.3048,   -2464.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(4.0\*0.3048,   -0.47);/speedTable.put(4.0*0.3048,   -2632.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(5.0\*0.3048,   -0.50);/speedTable.put(5.0*0.3048,   -2800.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(6.0\*0.3048,   -0.51);/speedTable.put(6.0*0.3048,   -2856.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(7.0\*0.3048,   -0.52);/speedTable.put(7.0*0.3048,   -2912.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(8.0\*0.3048,   -0.53);/speedTable.put(8.0*0.3048,   -2968.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(9.0\*0.3048,  -0.57);/speedTable.put(9.0*0.3048,    -3192.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(10.0\*0.3048,  -0.61);/speedTable.put(10.0*0.3048,   -3416.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(11.0\*0.3048,  -0.65);/speedTable.put(11.0*0.3048,   -3640.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(12.0\*0.3048,  -0.67);/speedTable.put(12.0*0.3048,   -3752.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(13.0\*0.3048,  -0.70);/speedTable.put(13.0*0.3048,   -3920.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(14.0\*0.3048,  -0.72);/speedTable.put(14.0*0.3048,   -4032.0);/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/speedTable.put(15.0\*0.3048,  -0.75);/speedTable.put(15.0*0.3048,   -4200.0);/g' src/main/java/frc/robot/Constants.java

sed -i '' 's/public static final double forwardDutyCycle = 0.2;/public static final double forwardRPM = 1120.0;/g' src/main/java/frc/robot/Constants.java
sed -i '' 's/public static final double reverseDutyCycle = -0.15;/public static final double reverseRPM = -840.0;/g' src/main/java/frc/robot/Constants.java
