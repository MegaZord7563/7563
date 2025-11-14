# :high_brightness: 2025 Megazord Robot Code

Welcome to the official code repository for the **2025 Megazord Robot (FRC Team 7563)**. This documentation provides an overview of the robot's subsystems, control systems, and how to interact with the codebase.


## 🤖  Robot Systems Overview

### 🌟Swerve Drive

O **Swerve Drive** permite movimentos precisos e omnidirecionais, permitindo que o robô navegue pelo campo com agilidade. O código é dividido em três componentes principais:

### 1. **SwerveModules**

Esta parte do código controla a saída de cada roda individualmente. Cada módulo é responsável pela direção e velocidade de uma roda. A configuração dos módulos é mostrada na imagem abaixo:

<img width="1796" height="768" alt="image" src="https://github.com/user-attachments/assets/a81d412d-fe94-47f3-b7cb-d32492d0994b" />


### 2. **Swerve Subsystem / Command**

Aqui, o código combina as saídas das rodas e cria **ChassisSpeeds** para controlar o movimento do robô. As entradas do joystick são processadas e convertidas em velocidades de saída do robô.

<img width="1497" height="870" alt="image" src="https://github.com/user-attachments/assets/154cf51f-9893-4e8c-91ac-a3a7db760426" />


### 3. **Classe Swerve Module**

Um acionamento Swerve consiste em 4 módulos, cada um contendo uma roda e dois motores:

**Drive Motor:**
- Controla a velocidade da roda.
- Motores Kraken com encoder incremental incorporado.

**Steering Motor:** 
- Controla a direção da roda.
- Motores Kraken com encoder absoluto CAnCoder.

### 4. **Swerve Kinematics**

A biblioteca `SwerveDriveKinematics` cuida do cálculo complexo no controle das velocidades e ângulos de cada módulo Swerve, transformando as velocidades desejadas do chassi em objetos SwerveModuleState individuais. 
Cada SwerveModuleState especifica a velocidade e o ângulo desejados para um único módulo swerve. 
Nosso código então envia esses comandos SwerveModuleState para cada módulo, que controla seus motores de acordo para atingir o movimento pretendido.

<img width="505" height="476" alt="image" src="https://github.com/user-attachments/assets/540b0736-524f-4f80-92fe-1d37d48b8c49" />

A cinemática do robô precisa ser atribuída corretamente, pois é por meio destes valores que a biblioteca calcula as velocidades e ângulos de cada módulo Swerve.
#### Localização de cada módulo

```java
       /**********************************************************************
       * Swerve Drive Object - It specifies the location of each swerve     *
       * module on the robot this way the wpi library can construct the     *
       * geometry of our robot setup and do all the calculations            *
       * @see  Modules Location: FL= +X,+Y; FR= +X,-Y; BL=-X, +Y; BR=-X, -Y,*
       **********************************************************************/
      public static final SwerveDriveKinematics kDriveKinematics = 
      new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), 
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) 
                                ); 
```


#### Cinemática do robô

- Distância em metros de centro a centro entre as rodas dianteiras.
- Distância em metros de centro a centro entre as rodas dianteiras e traseiras.
- Diâmetro das rodas em metros.
- `Offset` da posição absoluta inicial da rodas.
- `Offset` da posição inicial da roda no `Chassis`.

#### Distância das rodas
```java
         /**************************************
         * Specify the kinematics
         * # TO DO - According to your robot
         * ************************************/
      public static final double kTrackWidth =Units.inchesToMeters(21);
      // Distance between right and left wheels
      public static final double kWheelBase = Units.inchesToMeters(21);
      // Distance between front and back wheels
      public static final double kDriveRadius = Math.hypot(kTrackWidth/2, kWheelBase/2);
```


#### Offset do Chassis

Determina o sentido de rotação correta de cada motor

Por exemplo: Posicione todas as rodas alinhada para frente com todas as engrenagens chanfradas apontando para o centro do robô.
              Teremos um off set do Chassis conforme o código abaixo:

 ```java
      /**
      * kFrontLeftChassisAngularOffset = 0: This means that when your front left module's turning motor is at its zero position, 
      * the module is pointing straight forward relative to the chassis.
      * kFrontRightChassisAngularOffset = 0: This suggests your front right module is also pointing forward when its turning motor 
      * is at zero.
      * kBackLeftChassisAngularOffset = Math.PI: This means that your back left module is pointing 180 degrees from the front left 
      * module. So, when its turning motor is at zero, the module is pointing straight backward relative to the chassis.
      * kBackRightChassisAngularOffset = Math.PI: This means that your back right module is also pointing straight backward 
      * when its turning motor is at zero.
      ******/

      public static final double kFrontLeftChassisAngularOffset = 0;//
      public static final double kFrontRightChassisAngularOffset = Math.PI;
      public static final double kBackLeftChassisAngularOffset = 0;
      public static final double kBackRightChassisAngularOffset = Math.PI;
``` 

#### Offset da posição absoluta de cada roda

- Medida absoluta nesta mesma posição inicial em `radianos`. Pode ser em graus também ,  mas precisa converter usando a biblioteca `Units`como está no exemplo.
- Garante o perfeito alinhamento e movimentação do robô.

##### Calibração

- Posicione as rodas perfeitamente alinhadas com as engrenagens chanfradas apontando para o centro do robô.(sugestão)
- Leia o ângulo que cada encoder absoluto retorna
- Atribua os valores como offset para cada roda no seu código.

  ```java
      /**SINTONIA DAS RODAS
       * FL => FRONT LEFT 
       * FR => FRONT RIGHT
       * BL => BACK LEFT 
       * BR => BACK RIGHT
       * # TO DO - According to your robot
       */

      public static final Rotation2d angleOffsetFLTurning = Rotation2d.fromDegrees(-4.85);
      public static final Rotation2d angleOffsetFRTurning = Rotation2d.fromDegrees(-77.43);
      public static final Rotation2d angleOffsetBLTurning = Rotation2d.fromDegrees(142.207);
      public static final Rotation2d angleOffsetBRTurning = Rotation2d.fromDegrees(-91.14);

  ```
  
#### Método importantes na classe `Swerve Module`

#### 1.`SwerveModulePosition getPosition()`
  
```java
/**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   *
   */

  public SwerveModulePosition getPosition() 
  {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    
  return new SwerveModulePosition(
                                      this.getDrivePositionMeters(),
                                      new Rotation2d(this.getTurningPositionRad()-m_chassisAngularOffset)
                                       );
  }
```
#### 2.`SwerveModuleState`

```java
/**
   * Returns the current state of the module, drive speed in m/s 
   * and angle in Rotation 2d
   * @return The current state of the module.
   *
   */

  public SwerveModuleState getState() 
  {
    return new SwerveModuleState(this.getDriveVelocityMPS(), new Rotation2d(this.getTurningPositionRad()- m_chassisAngularOffset));
    
  }
```
#### 3.`public void setDesiredState(SwerveModuleState state) `

```java

/**
   * Sets the desired state for each module.
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) 
  {
      
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = state.speedMetersPerSecond;
    correctedDesiredState.angle = state.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the state to ensure the shortest path to the desired angle.
    
    correctedDesiredState.optimize(new Rotation2d(this.getTurningPositionRad()));

    
      
    // Set the drive motor to the desired speed.
    double driveVelocityRPS = Conversions.MPSToRPS(correctedDesiredState.speedMetersPerSecond, 
                                                  ModuleConstants.kWheelCircumferenceMeters,
                                                  ModuleConstants.kDriveMotorGearRatio);

    //set the drive motor to the desired speed.                                         
    driveMotor.setControl( m_velocityDrive.withVelocity(driveVelocityRPS)); 
    
    //set the turning motor to the desired angle.
    turningMotor.setControl(m_positionTurning.withPosition(correctedDesiredState.angle.getRotations()));

    
    // Set the desired state.
    m_desiredState = state;
  }

```
### 4. **Classe Swerve Subsystem**

- Classe que constroe todos os módulos swerve
- Formada por:
  - Swerve module - FL, FR, BL e BR
  - IMU - giroscópio
  - Odometria neste caso usando o Pose Estimator , deve ser atualizada no método `Periodic`.

```java
//Array of modules
  private final SwerveModule[] modules = {frontLeftModule,frontRightModule,backLeftModule,backRightModule};

//Create Pigeon2 gyro
  private final Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonPort, "rio");
  // Create a new SwerveDrivePoseEstimator
  private final SwerveDrivePoseEstimator m_poseEstimator = 
                                          new SwerveDrivePoseEstimator(
                                              DriveConstants.kDriveKinematics,
                                              Rotation2d.fromDegrees(allianceHeading),//gyro.getRotation2d(),
                                              new SwerveModulePosition[] {
                                                frontLeftModule.getPosition(),
                                                frontRightModule.getPosition(),
                                                backLeftModule.getPosition(),
                                                backRightModule.getPosition()
                                              },
                                              new Pose2d(),
                                              VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                                              VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
```
#### 4.1 ChassisSpeed

É o objeto que por meio das velocidades:
       
       Vx -> em m/s
       Vy -> em m/s
       VΦ -> em rad/s
Calcula o movimento de translação e rotação do robô , distribuindo as velocidades lineares e angulares para cada módulo de acordo com a posição e `cinemática`.

```java
/**
   * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
   * Path Planner uses
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) 
  {
    // Convert chassis speeds to individual module states
    SwerveModuleState[] swerveModuleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
       
    // Output each module states to wheels
    this.setModuleStates(swerveModuleStates);
    
  }
```

#### 4.2 Swerve Module State

É o objeto que converte os valores do `ChassisSpeed` para atribuir a velocidade e posição de cada módulo.

```java
 /**
 * Sets the swerve ModuleStates.
 *
 * @param desiredStates The desired SwerveModule states.
 */
  public void setModuleStates (SwerveModuleState [] desiredStates)
  { 

    //normalize the wheel speeds ;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond); 
    
    // Output Module States to each one - Convert chassis speeds to individual module states
    for (int i = 0; i < modules.length; i++) 
    {
        modules[i].setDesiredState(desiredStates[i]);
      
    }
    
  }
```
