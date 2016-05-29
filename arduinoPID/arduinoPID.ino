class TEMP{

  public:

    int
      sensor;

    float
      filtered = 100.0,
      temperature[100],
      temperaturefiltered;

    TEMP (int _sensor){
      sensor= _sensor;
    }

    float readTemp(){
      
      for (int i = filtered - 1; i > 0; i--) {
        temperature[i] = temperature[i - 1];
      }
      temperature[0] = (analogRead(sensor) * 0.00488758) * 100.0;
      temperaturefiltered = 0;
  
      for (int i = 0; i < filtered; i++) {
        temperaturefiltered = temperaturefiltered + temperature[i];
      }

      temperaturefiltered = temperaturefiltered / filtered;

      return temperaturefiltered;
    
    }
};

class HIS{

  public:

    bool
      control;
      
    float
      offsetHis,
      sample,
      setPoint;

    HIS (float _offsetHis){
      offsetHis = _offsetHis;
    }

    void addNewSample(float _sample){
      sample = _sample;
    }

    void setSetPoint(double _setPoint){
      setPoint = _setPoint;
    }
    
    double process(){
      if (sample <= setPoint - offsetHis){
        control = HIGH;
      }
      if (sample >= setPoint + offsetHis){
        control = LOW;
      }

      return control;
    }
};

class PID{

  public:
    
    double
      error,
      sample,
      lastSample,
      kP,
      kI,
      kD,
      P,
      I,
      D,
      pid,
      setPoint;

    long
      lastProcess;

    PID (double _kP, double _kI, double _kD){
      kP = _kP;
      kI = _kI;
      kD = _kD;  
    }

    void addNewSample(double _sample){
      sample = _sample;
    }

    void setSetPoint(double _setPoint){
      setPoint = _setPoint;
    }

    double process(){
      error = sample - setPoint;
      float deltaTime = (millis() - lastProcess) / 1000.0;
      lastProcess = millis();
      //P
      P = error * kP;
      //I
      I += (error * kI) * deltaTime;
      //D
      D = (sample - lastSample) * kD / deltaTime;
      lastSample = sample;
      //PID
      pid = P + I + D;

      return pid;
    }
    
};


  #define       pSensor     A0
  #define       pLed        2
  #define       pControle   3
  #define       setPointMin 20.0
  #define       setPointMax 50.0

  HIS           temperatureHIS (1.0);
  PID           temperaturePID (1.0, 0.05, 0.05);
  TEMP          temperatureTEMP (pSensor);

  int           controlePWM = 0;
  int           mode = 0;
  float         setpoint = 0.0;
  unsigned long timercontrol = 0;

  String        serialmessage;
  
void setup(void) {
  
  Serial.begin(115200);
  
  pinMode(pSensor,    INPUT);
  pinMode(pControle, OUTPUT);
  pinMode(pLed,      OUTPUT);

  Serial.print('\n');
  Serial.print(F("Selecione o modo de controle"));
  Serial.println('\n');
  
  Serial.print(F("Digite '1' para Histerese e '2' para PID."));
  Serial.print('\n');
}

float getvaluef(String pString){
   
  char vTempValue[10];
  pString.toCharArray(vTempValue,sizeof(vTempValue));
  return atof(vTempValue);
  
}

void msgEscala(void){
  Serial.print('\n');
  Serial.print(F("Digite um valor entre "));
  Serial.print(setPointMin, 2);
  Serial.print(" e "); 
  Serial.print(setPointMax, 2);
  Serial.print(F(" para configurar o setpoint.")); 
  Serial.print('\n'); 
}

void loop(void) {

  unsigned long timer = millis();
  float         setpointmessage;
  double        temperature = temperatureTEMP.readTemp();
  String        modelabel[] = {"Controle Histerese", "Controle PID"};
  
  if (Serial.available() > 0 && mode == 0){
      serialmessage = Serial.readStringUntil('\n');
      Serial.flush(); 

    if (serialmessage.toInt() > 0 && serialmessage.toInt() < 3){
      mode = serialmessage.toInt();
      serialmessage = "";

      Serial.print('\n');
      Serial.print(modelabel[mode - 1]);
      Serial.print('\n');
      msgEscala();
    }
    else{
      Serial.print('\n');
      Serial.print(F("O valor digitado para o modo de controle não é valido."));
      Serial.print('\n');
      Serial.print(F("Por favor, digite um valor válido."));
      Serial.print('\n');
      Serial.print(F("Digite '1' para Histerese e '2' para PID."));
      Serial.print('\n');
    }
  }
  
  if (Serial.available() > 0 && mode != 0 && setpoint == 0.0){
      serialmessage = Serial.readStringUntil('\n');
      Serial.flush();  

    setpointmessage = getvaluef(serialmessage);
    
    if (setpointmessage >= setPointMin && setpointmessage <= setPointMax){
      setpoint = setpointmessage;
      timercontrol = millis();
      serialmessage = "";
      Serial.print('\n');
    }
    else{
      Serial.print('\n');
      Serial.print(F("O valor digitado para o setpoint esta fora da escala."));
      Serial.print('\n');
      Serial.print(F("Por favor, digite um valor válido."));
      msgEscala();
    }
        
  }

  switch (mode){
    case 0 :
      break;
    case 1:
      if (setpoint > 0.0){
        
        temperatureHIS.setSetPoint(setpoint);
        temperatureHIS.addNewSample(temperature);
        
        digitalWrite(pControle, temperatureHIS.process());
        digitalWrite(pLed,      temperatureHIS.process());
        
      }
      break;
    case 2:
      if (setpoint > 0.0){
        
        temperaturePID.setSetPoint(setpoint);
        temperaturePID.addNewSample(temperature);
        controlePWM = temperaturePID.process();
      
        analogWrite(pControle, controlePWM);
        analogWrite(pLed,      controlePWM);
        break;
      }
  }

  if (setpoint > 0.0){
    if (timer > timercontrol + 500){
      timercontrol = millis();
      Serial.print(F("Temperatura atual: "));
      Serial.print(temperature);
      Serial.print(F(" C   ---   Setpoint atual: "));
      Serial.print(setpoint);
      Serial.print(" C   ---   Modo: ");
      Serial.print(modelabel[mode - 1]);
      Serial.print('\n');
    }
  }
}
