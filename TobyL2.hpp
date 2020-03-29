#ifndef TRANSPORTS_GPS_TOBY_L2_INCLUDED
#define TRANSPORTS_GPS_TOBY_L2_INCLUDED

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Transports
{
  namespace GSMTobyL2
  {

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
      initTobyL2 (const std::string apn , const std::string pin , const float rssi_timer , const float ntwk_timer)
      {
        setAirplaneMode(true);
        //! Get IMEI
        m_IMEI = getIMEI();
        //! Get IMSI
        m_IMSI = getIMSI();
        //! Set Verbose Output
        setErrorVerbosity(2);
        //! Set PIN if needed
        if (!pin.empty())
        { //! Set SIM PIN here
          setPIN(pin);
        }
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
          m_rssi_querry_timer.reset();
        }
        if (m_ntwk_querry_timer.overflow())
        {
          m_ntwk_querry_timer.overflow();
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
      setPIN(const std::string pin)
      {
        std::string bfr = readValue("+CPIN?");
        if (bfr == "+CPIN: READY")
          return;

        if (bfr == "+CPIN: SIM PIN")
        {
          sendAT(String::str("+CPIN=%s", pin.c_str()));
          expectOK();
        }
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
          sendAT("+CFUN=1,1");
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
    };
  }
}
#endif