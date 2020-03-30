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
      }

      ~TobyL2()
      {

      }

      void 
      initTobyL2(const std::string apn , const std::string pin , const float rssi_timer , const float ntwk_timer)
      {
        m_task->inf("Initializing the Modem");
        flushInput();
        // Perform initialization.
        setReadMode(READ_MODE_LINE);
        start();
        sendInitialization();
        setEcho(false);
        setAirplaneMode(true);
        //! Get IMEI
        m_IMEI = getIMEI();
        m_task->inf("IMEI : %s " , m_IMEI.c_str());
        //! Set Verbose Output
        setErrorVerbosity(2);
        //! Set PIN if needed       
        setPIN(pin);
        //! Get IMSI
        m_IMSI = getIMSI();
        m_task->inf("IMSI : %s " , m_IMSI.c_str());
        //! Set APN to connect to
        setAPN(apn);
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
              if (checkSIMStatus() == "+CPIN: READY")
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
              m_task->inf("Registration Value %d" , ntwk_register);
              if (ntwk_register == 1 || ntwk_register == 5 )
              {
                m_modem_state++;
              }
              else if (ntwk_register == 0)
              {
                //! Manually Start Registration Again
                startNetworkRegistration();
              }
              break;
            }

            case NETWORK_REGISTRATION_DONE:
            {
              int rat_type = getRATType();
              m_task->inf("RAT Type %d " , rat_type);

              if (rat_type > 0 && rat_type < 7)
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
              else if (rat_type == 7)
              {
                //! Connected to LTE network Do not thing Modem auto connents to internet
                m_modem_state++;
              }
              break;
            }

            case PDP_CONTEXT_ATTACHED:
            {
              //! Check PDP Context
              int status =  getPDPContext();
              m_task->inf("CGACT Status %d " , status);
              if (status == 1)
              {
                m_modem_state++;
              }
              else if (status == 0)
              {
                m_modem_state = INITIAL_STATE;
              }
              break;
            }

            case NETWORK_CONNECTION_OK:
            {
              m_modem_state = INITIAL_STATE;
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


      void
      activatePDPContext()
      {
        sendAT("+CGACT=1,1");
        expectOK();
      }

      int 
      getPDPContext()
      {
        int status = -1 , cid = -1;
        sendAT("+CGACT?");
        //! +CGACT: 1,1
        std::string line = readLine();
        expectOK();
        std::sscanf(line.c_str(), "+CGACT: %d,%d", &cid, &status);
        return status;
      }

      void 
      setPIN(const std::string pin)
      {
        std::string bfr =  checkSIMStatus();
        if (bfr == "+CPIN: READY")
        {
          m_task->inf("PIN ready");
          return;
        }

        if (bfr == "+CPIN: SIM PIN")
        {
          sendAT(String::str("+CPIN=%s", pin.c_str()));
          expectOK();

        }
        m_task->err("Exitting pin");
      }

      std::string 
      checkSIMStatus()
      {
        return readValue("+CPIN?");
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
      startNetworkRegistration()
      {
        sendAT("+COPS=0");
        expectOK();
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
          //! Set Full Funtionality Mode with silent reset
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