#ifndef TRANSPORTS_GPS_TOBY_L2_INCLUDED
#define TRANSPORTS_GPS_TOBY_L2_INCLUDED

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Transports
{
  namespace GSMTobyL2
  {
    enum State{
      INITIAL_STATE             = 0,
      SIM_CARD_READY            = 1,
      NETWORK_REGISTRATION_DONE = 2,
      PDP_CONTEXT_ATTACHED      = 3,
      NETWORK_CONNECTION_OK     = 4
    };

    using DUNE_NAMESPACES;
    class TobyL2 : public HayesModem
    {
    public:
      //! Parent task.
      Tasks::Task* m_task;
      //! Timer for RSSI Querry
      DUNE::Time::Counter<float> m_rssi_querry_timer;
      //! Timer for Network Check
      DUNE::Time::Counter<float> m_ntwk_querry_timer;
      //! IMEI number of the modem
      std::string m_IMEI;
      //! IMSI number of the SIM card
      std::string m_IMSI;
      //! Phone number of the SIM card
      std::string m_phone_number;
      //! Current State of Modem
      uint8_t m_modem_state = INITIAL_STATE;
      //! Signal Strength
      float m_rssi;

      TobyL2(Tasks::Task* task , SerialPort* uart):
      HayesModem(task, uart),
      m_task(task)
      {
        setLineTrim(true);
        setReadMode(READ_MODE_LINE);
        start();
      }

      ~TobyL2()
      {
        
      }

      void 
      initTobyL2(const std::string apn , const std::string pin , const float rssi_timer , const float ntwk_timer)
      {
        m_task->inf("Initializing the Modem");
        setEcho(false);
        setAirplaneMode(true);
        //! Get IMEI
        m_IMEI = getIMEI();
        m_task->inf("IMEI : %s " , m_IMEI.c_str());
        //! Set Verbose Output
        setErrorVerbosity(2);
        //! Set PIN if needed       
        if (setPIN(pin) > -1)
        {
          //! Get IMSI
          m_IMSI = getIMSI();
          m_task->inf("IMSI : %s " , m_IMSI.c_str());
        }
        //! Set APN to connect to
        setAPN(apn);
        //! Configure SMS Properties
        configureSMS(); 
        //! Remore from Airplane Mode
        setAirplaneMode(false);
        setRssiTimer(rssi_timer);
        setNtwkTimer(ntwk_timer);
      }

      void 
      updateTobyL2()
      {
        if (m_rssi_querry_timer.overflow())
        {
          if (m_modem_state >= NETWORK_REGISTRATION_DONE )
          {
            queryRSSI();
            m_task->inf("Current Signal Strength %.2f%% " , m_rssi); 
          }
          m_rssi_querry_timer.reset();
        }


        if (m_ntwk_querry_timer.overflow())
        {
          switch(m_modem_state)
          {
            case INITIAL_STATE:
            {
              if (checkSIMStatus() == 1)
              {
                m_modem_state++;;
              }
              else
              {
                m_task->err("SIM card Error");
              }
              break;
            }

            case SIM_CARD_READY:
            {
              int ntwk_register = checkNetworkRegistration();
              //! ntwk_registration 1 is registered to home netowrk
              //! ntwk_registration 5 is registered to roaming network
              m_task->inf("Network Registration Value %d" , ntwk_register);
              if (ntwk_register == 1 || ntwk_register == 5 )
              {
                m_modem_state++;
              }

              break;
            }

            case NETWORK_REGISTRATION_DONE:
            {
              int rat_type = getRATType();
              m_task->inf("Radio Access Technology Type %d " , rat_type);

              if (rat_type > 0 && rat_type < 7 && !checkPDPContext())
              {
                //! RAT 1 = GSM COMPACT
                //! RAT 2 = UTRAN
                //! RAT 3 = GSM/GPRS with EDGE availability
                //! RAT 4 = UTRAN with HSDPA availability
                //! RAT 5 = UTRAN with HSUPA availability
                //! RAT 6 = UTRAN with HSDPA and HSUPA availability
                //! Need to connect to Internat manually
                activatePDPContext();
                m_modem_state++;
              }
              else
              {
                //! Connected to LTE network Do not thing Modem auto connents to internet
                m_modem_state++;
              }
              break;
            }

            case PDP_CONTEXT_ATTACHED:
            {
              //! Check PDP Context
              bool status =  checkPDPContext();
              m_task->inf("PDP Context Status %d " , status);
              if (status == true)
              {
                m_modem_state++;
              }
              else if (status == false)
              {
                m_modem_state = INITIAL_STATE;
              }
              break;
            }

            case NETWORK_CONNECTION_OK:
            {
              bool status =  checkPDPContext();
              if ( status == false)
              {
                m_modem_state = INITIAL_STATE;
              }
              else
              {
                if (pingRemote("www.google.com") < 0)
                {
                  m_modem_state = INITIAL_STATE;
                }  
              }
              break;
            }
          }
          m_ntwk_querry_timer.reset();
        }
      }

      void 
      setRssiTimer(const float rssi_timer)
      {
        m_rssi_querry_timer.setTop(rssi_timer);
      }

      void 
      setNtwkTimer(const float ntwk_timer)
      {
        m_ntwk_querry_timer.setTop(ntwk_timer);
      }


    private:

      int
      pingRemote(std::string remote)
      {
        int round_trip_time = -1;
        sendAT("+UPING=\""+remote+"\",1,32,2000,255");
        std::string line = readLine();
        if (line == "OK")
        {
          line = readLine();
          //+UUPING: 1,32,\"www.google.com\","172.217.23.100",53,260
          std::vector<std::string> tokens;
          std::istringstream ss(line);
          std::string token;
          while(std::getline(ss, token, ','))
          {
            tokens.push_back(token);
          }
          m_task->inf("Ping Value %s " , tokens[5].c_str());
          round_trip_time = std::stoi(tokens[5]);
        }
        return round_trip_time;
      }


      void
      configureSMS()
      {
        //! Set Charater set to IRA
        sendAT("+CSCS=\"IRA\"");
        expectOK();
        //! Select procedure to indicate new Message
        sendAT("+CNMI=2,2");
        expectOK();
        //! Select Text mode as prefered message format
        sendAT("+CMGF=1");
        expectOK();
      }


      void
      activatePDPContext()
      {
        sendAT("+CGACT=1,1");
        expectOK();
        //! Map PSD profile 0 to 1
        sendAT("+UPSD=0,100,1");
        expectOK();
        //! Set PDP Type IPv4
        sendAT("+UPSD=0,0,0");
        expectOK();
        //! Activate Internal PSD
        sendAT("+UPSDA=0,3");
        expectOK();
        std::string line = readLine();
      }

      bool 
      checkPDPContext()
      {
        uint8_t ok = 0; 
        std::vector<std::string> arr;
        sendAT("+CGACT?");
        //! +CGACT: 1,1
        while (!ok)
        {
          std::string line = readLine();
          if (line == "OK")
          {
            ok = 1;
          }
          else
          {
            arr.push_back(line);
          }
          
        }

        for (uint8_t i = 0 ; i < arr.size() ; i++)
        {
          int status = -1 , cid = -1;
          std::sscanf(arr[i].c_str(), "+CGACT: %d,%d", &cid, &status);
          if (status > 0)
          {
            //! Atleast one PDP context is active
            return true;
          }
        }        
        return false;
      }

      int
      setPIN(const std::string pin)
      {
        int value =  checkSIMStatus();
        if (value == 1)
        {
          return 1;
        }
        else if (value == -2)
        {
          sendAT(String::str("+CPIN=%s", pin.c_str()));
          expectOK();
          return 1;
        }
        else
        {
          return -1;
        }
      }

      int 
      checkSIMStatus()
      {
        std::string bfr = readValue("+CPIN?");

        if (bfr == "+CPIN: READY")
        {
          return 1;
        }
        else if (bfr == "+CPIN: SIM PIN")
        {
          return -2;
        }

        return -1;
      }

      int
      getRATType()
      {
        sendAT("+COPS?");
        std::string line = readLine();
        expectOK();
        if (line.find("+COPS") != std::string::npos)
        {
          std::vector<std::string> tokens;
          std::istringstream ss(line);
          std::string token;
          while(std::getline(ss, token, ','))
          {
            tokens.push_back(token);
          }
          return std::stoi(tokens[3]);
        }
        return -1;
      }

      int
      checkNetworkRegistration()
      {
        int n = -1 , stat = -1;
        sendAT("+CREG?");
        std::string line = readLine();
        expectOK();
        std::sscanf(line.c_str(), "+CREG: %d,%d", &n, &stat);
        return stat;
      }

      void 
      setAPN(const std::string apn)
      {
        sendAT("+CGDCONT=1,\"IP\",\"" + apn + "\"");
        expectOK();
        sendAT("+UCGDFLT=1,\"IP\",\"" + apn + "\"");
        expectOK();
      }

      void 
      setAirplaneMode(bool enable)
      {
        if (enable)
        {
          sendAT("+CFUN=4");
          expectOK();
        }
        else 
        {
          //! Set Full Funtionality Mode
          sendAT("+CFUN=1");
          expectOK();
        }
      }

      void
      setErrorVerbosity(unsigned value)
      {
        sendAT(String::str("+CMEE=%u", value));
        expectOK();
      }

      std::string
      getIMSI(void)
      {
        return readValue("+CIMI");
      }

      void
      queryRSSI(void)
      {
        sendAT("+CSQ");
        std::string line = readLine();
        int rssi = -1;
        int ber = 0;
        if (std::sscanf(line.c_str(), "+CSQ: %d,%d", &rssi, &ber) == 2)
        {
          expectOK();
          m_rssi = convertRSSI(rssi);
        }
      }

      float
      convertRSSI(int rssi)
      {
        float cvt = -1.0f;

        if (rssi >= 0 && rssi <= 9)
          cvt = (rssi / 9.0) * 25.0f;
        else if (rssi >= 10 && rssi <= 14)
          cvt = 25.0f + (((rssi - 10) / 4.0f) * 25.0f);
        else if (rssi >= 15 && rssi <= 19)
          cvt = 50.0f + (((rssi - 15) / 4.0f) * 25.0f);
        else
        {
          if (rssi >= 31)
            rssi = 31;

          cvt = 75.0f + (((rssi - 20) / 11.0f) * 25.0f);
        }

        return cvt;
      }
    };
  }
}
#endif