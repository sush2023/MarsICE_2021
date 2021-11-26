#include <arduino-timer.h>

class CurrentMon{
    private:

        struct readingDat
        {
            int analogPin;
            int * i;
            uint16_t * arr;
            int sampleNum;
            double * Irms;
        };

        int analogPin;
        double offset;
        double scale;

        double Irms;
        uint16_t* readings;

        int sampleNum;
        int sampleTime;

        Timer<10, micros> timer;

        bool localTimer = false;

    public:

        CurrentMon(double analogPin,double offset, double scale, int sampleNum, int sampleTime, Timer<10, micros> t){
            this->analogPin = analogPin ;
            this->offset = offset;
            this->scale  = scale ;
            this->sampleTime = sampleTime;
            this->sampleNum = sampleNum;

            this->timer = t;

        }

        CurrentMon(double offset, double scale):CurrentMon(0, offset, scale, 250, 2, *(new Timer<10, micros>)){
            Timer<10, micros> t;
            localTimer = true;
        }

        ~CurrentMon(){
            delete(readings);
            if(localTimer){
                delete(&timer);
            }
        }

        float getNewIrms(){
            updateIrms();
            return getLastIrms();
        }

        float getLastIrms(){
            return (Irms+offset)*scale;;
        }

        bool static getReading(readingDat *args){
            args->arr[*(args->i)] = analogRead(args->analogPin);
            return ++*(args->i) <= (args->sampleNum);
        }

        bool updateIrms(){
            readings = new uint16_t[sampleNum];

            size_t tasksRunning = timer.size();

            int i = 0;
            readingDat rd;
            rd.analogPin = analogPin;
            rd.i = &i;
            rd.arr = readings;
            rd.sampleNum = sampleNum;
            rd.Irms = &Irms;

            timer.every(sampleTime, &CurrentMon::getReading, (void *)&rd);
            
            timer.in(sampleTime*sampleNum+10, &CurrentMon::calcReading, (void *)&rd);
            
        }

        bool static calcReading(readingDat* dat){

            for (size_t i = 0; i < dat->sampleNum; i++)
            {
                *(dat->Irms) += dat->arr[i];
            }

            *(dat->Irms) /= dat->sampleNum;
            *(dat->Irms) = pow(*(dat->Irms), .5);

            delete(dat->arr);
        }



        void setSampleTime(int sampleTime){
            this->sampleTime = sampleTime;
        }

        void setSampleNum(int sampleNum){
            this->sampleNum = sampleNum;
        }
};
