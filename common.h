#define MAXDATASIZE 256
#define HEADER_LEN (sizeof(unsigned short)*3)
#define ID 0;
struct appdata{

       unsigned short id; //identificador
        unsigned short op; //codigo de operacion
        unsigned short len;                       /* longitud de datos */
       

        char data [MAXDATASIZE-HEADER_LEN];//datos


};
//operacion error
#define OP_ERROR            0xFFFF
//operaciones requeridas por central
#define OP_SALUDO           0x0001
#define OP_MOVE_WHEEL      0x0002

#define OP_STOP_WHEEL       0x0003

//operaciones cliente
#define OP_MESSAGE_RECIVE   0x0004
//saludo esta en ambas

//----------------parametros RObot-------------------
#define D 6.7//cm
#define R 3.2 //aprox
#define L 9.5 // cm distancia entre ruedas


     //     ---------------motor setup---------------*/
const int pinENA = 12;//señal de PWM
const int pinIN1 = 7;//indica sentido de giro
const int pinIN2 = 8;//indica sentido de giro

const int pinIN3 = 9;//indica sentido de giro
const int pinIN4 = 10;
const int pinENB = 11;//Señal de PWM

const int pinMotorD[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorI[3] = { pinENB, pinIN3, pinIN4 };

//-----------------contador encoder------------------------------------------------------------------------------------
const int             N=                  20;//Resolucion encoder       
const int             encoderD =          15;//pin de entrada de encoder derecha
const int             encoderI=           13;//pin de entrada de encoder izquierda
volatile unsigned     encoder_countD=      0;//cuenta los pulsos de encoder derecha
int                   encoder_countD_after=0;
int                   encoder_countI_after=0;
int                   dif_encoderD=0;
int                   dif_encoderI=0;
volatile unsigned     encoder_countI=      0;//cuenta los pulsos de encoder izquierda
volatile int          vueltaD=             0;//cuenta las vueltas que ha dado la rueda derecha
volatile int          vueltaI=             0;//cuenta las vueltas que ha dado la rueda izquierda
int                   valorD=              0;//media de tiempo

//----------T-------IEMPO ENTRE INTERRUPCIONES-----------------------------------------------------------------------//
volatile unsigned long startTimeI=         0;
volatile unsigned long timeAfterI=         0;
volatile unsigned      deltaTimeI;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long startTimeD=         0;
volatile unsigned long timeAfterD=         0;
volatile unsigned      deltaTimeD;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long timeStopD=          0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long timeStopI=          0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long deltaTimeStopD;
volatile unsigned long deltaTimeStopI;
//------------------------debounce-----------------------------------------------------------------------------------
//Se usara para evitar que cuenten pulsos debido a rebotes o ruido del sistema.
# define               TIMEDEBOUNCE        8 //(ms) es el tiempo minimo entre pulsos
volatile unsigned long timeAfterDebounceD= 0;
volatile unsigned long timeBeforeDebounceD=0;
volatile unsigned long deltaDebounceD=     0;

volatile unsigned long timeAfterDebounceI= 0;
volatile unsigned long timeBeforeDebounceI=0;
volatile unsigned long deltaDebounceI=     0;

//---------------------variables odometria------------------------------------------------------------------------------------
double                 distanciaI=0;
double                 distanciaD=0;

//---------------------VARIABLES DEL CONTROLADOR PID-----------------------------------------------------------------------------
//+++++++++++ variables internas del controlador++++++++++++++
double                   k=1;//se mide en milisegundos y sirve para establecer un tiempo de muestreo discreto
//++++++++++++++++++rueda derecha+++++++++++++++++
unsigned long         currentTimeD, previousTimeD=0;;
double                elapsedTimeD;
double                errorD=0, lastErrorD=0, cumErrorD=0, rateErrorD;
// ++++++++++rueda izquierda++++++++++++
unsigned long         currentTimeI, previousTimeI=0;;
double                elapsedTimeI;
double                errorI=0, lastErrorI=0, cumErrorI=0, rateErrorI;
// +++++++++++++++  ++++Constantes del controlador+++++++++++++++
double                kp=0, Ki=0.00007, Kd=0;
// ++++++++++++++++++variables externas del controlador++++++++++++++++++
double                Input, output;
//double                Setpoint;//se usa para indicar el valor deseado unidades en rad/s

//-------------------------GIROSCOPIO------------------------------------------------------------------------------------------------//
float x, y, z;
const float ERR_GIROSCOPE=3.05;
//------------------------feedforward Y PWM----------------------------------------------------//
int    PWM_D;
int    PWM_I;
double setpointWD;
double setpointWI;
double SetpointD,SetpointI,SetpointAnterior=0;//se usa para indicar el valor de referncia es temporal se debera usar uno para cada rueda
bool backD=false,backI=false;
