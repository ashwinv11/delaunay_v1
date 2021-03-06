// Number of Sensors
6 => int numberSensors; 
// Member Global Variables
int sensor[numberSensors]; // raw
int sensorClean[numberSensors]; // signal conditioning on IR sensor
Event irBang;  // threshold for IR sensor

// OSC Initialization
OscSend xmit;
"localhost" => string hostname;
12001 => int portOSC;
xmit.setHost(hostname, portOSC);

// Serial Handling 
SerialIO serial;
string line;
string stringInts[6];
int data[6];

fun void initSerial()
{
    
    SerialIO.list() @=> string list[];
    for( int i; i < list.cap(); i++ )
    {
        chout <= i <= ": " <= list[i] <= IO.newline();
    }
    serial.open(3, SerialIO.B9600, SerialIO.ASCII);
    
    1::second => now;
    
    spork ~ serialPoller();
}

// ***********************************************************
//
// SERIAL
// 
// ***********************************************************


fun void serialPoller(){
    while( true )
    {
        // Grab Serial data
        serial.onLine()=>now;
        serial.getLine()=>line;
        
        if( line$Object == null ) continue;
        
        0 => stringInts.size;
        
        // Line Parser
        if (RegEx.match("\\[([0-9]+),([0-9]+),([0-9]+),([0-9]+),([0-9]+),([0-9]+)\\]", line , stringInts))
        {      
            for( 1=>int i; i<stringInts.cap(); i++)  
            {
                // Convert string to Integer
                Std.atoi(stringInts[i])=>data[i-1];
            }
        }
        
        
        // Organize Sensor Data in Array sensor[]
        // sensors are in data 2, 3 & 4 
        for( 2 => int i; i <= 4; i++)
        {
            data[i] => sensor[i-2];
        }   
    }
}



// ***********************************************************
//
// SIGNAL CONDITIONING
// 
// ***********************************************************
fun void lowPass(int sensorIndex, dur sampleTime)
{
    sensor[sensorIndex] => int lastSensor;
    while( true ) 
    {
        sampleTime => now;
        (sensor[sensorIndex] + lastSensor) / 2 => sensorClean[sensorIndex];   
        sensor[sensorIndex] => lastSensor;
    }
}



fun void irBanger(int sensorIndex, dur sampleTime, int Threshold)
{
    sensorClean[sensorIndex] => int lastSensor;
    while( true ) 
    {
        sampleTime => now;
        sensorClean[sensorIndex] - lastSensor => int Derivative;
        
        if( Derivative > Threshold )
        {
            irBang.broadcast();
            //<<<" Bang... IR " >>>;
            500::ms => now;  //allow one bang every 500 ms. 
        } 
        
    }
}

fun void initSignalCondition()
{
    // deal with this
    spork ~ irBanger(2,10::ms, 700);      
    spork ~ lowPass(2, 10::ms);
    spork ~ lowPass(1, 10::ms);
}

// ***********************************************************
//
// OSC
// 
// ***********************************************************
fun void oscSensor(dur samplerate)
{
    
    // Create typetag based on number sensors
    string typetag;
    for( 0 => int i; i < sensor.cap(); i++)
    {
        "i" +=> typetag;
    }
    
    while( true )
    {
        // create OSC message
        xmit.startMsg("/pot1", typetag);
        for( 0 => int i; i < sensor.cap(); i++)
        {
            data[0] => xmit.addInt;
        } 

        xmit.startMsg("/pot2", typetag);
        for( 0 => int i; i < sensor.cap(); i++)
        {
            data[1] => xmit.addInt;
        }

        xmit.startMsg("/topright", typetag);
        for( 0 => int i; i < sensor.cap(); i++)
        {
            data[2] => xmit.addInt;
        }

        xmit.startMsg("/topleft", typetag);
        for( 0 => int i; i < sensor.cap(); i++)
        {
            data[3] => xmit.addInt;
        }  
        
        samplerate => now;
    }
}


fun void initOsc()
{
  spork ~ oscSensor(1000::ms);
}

// STUFF

// initialize instrument
initSerial();
2::second => now;
initOsc();
initSignalCondition();

// n channels
dac.channels() => int N;
// print
<<< "dac has", N, "channels..." >>>;

// delays
DelayL d[N];
// gains
Gain g3[N];

// our patch - feedforward part
adc => Gain g;
adc => Gain g2 => GVerb v => dac;

// set delays
for( int i; i < N; i++ )
{
    // feedfoward
    g => d[i] => dac.chan(i);
    // feedback
    d[i] => g3[i] => d[i];
    // detune
    30::ms + Math.random2f(-15,15)::ms => d[i].max => d[i].delay;
    .95 => g3[i].gain;
}

// set parameters
0.25 => g.gain;
0.05 => g2.gain;
0.25 => v.gain;

// MOVE THIS -- This will be the Art part
while (true)
{
    //<<< data[0], "\t", sensorClean[0], "\t", data[1], "\t", sensorClean[1] >>>;
    //<<< data[0], "\t",  data[1], "\t", data[2], "\t", data[3], "\t",  data[4], "\t", data[5], "\t", sensorClean[1], "\t", data[4], "\t", sensorClean[2] >>>;
    //<<< data[4], "\t", data[5] >>>;

    0.005 * data[4] => g.gain;
    0.005 * data[5] => g2.gain;
    v.tail(100 / (data[4] + 1));

    .05::second => now;
}
