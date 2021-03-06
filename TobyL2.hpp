#ifndef TRANSPORTS_GPS_TOBY_L2_INCLUDED
#define TRANSPORTS_GPS_TOBY_L2_INCLUDED
// ISO C++ 98 headers.
#include <cstring>
#include <queue>
#include <cstddef>

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

    //! SMS terminator character.
    static const char c_sms_term = 0x1a;
    //! SMS input prompt.
    static const char* c_sms_prompt = "\r\n> ";
    //! Size of SMS input prompt.
    static const unsigned c_sms_prompt_size = std::strlen(c_sms_prompt);


    using DUNE_NAMESPACES;
    class TobyL2 : public HayesModem
    {
    public:
      struct SmsRequest
      {
        // Request id.
        uint16_t req_id;
        // Source address.
        uint16_t src_adr;
        // Source entity id.
        uint8_t src_eid;
        // Recipient.
        std::string destination;
        // Message to send.
        std::string sms_text;
        // Deadline to deliver the
        double deadline;
        // Higher deadlines have less priority.

        bool
        operator<(const SmsRequest& other) const
        {
          return deadline > other.deadline;
        }
      };

      struct SMS
      {
        // Recipient.
        std::string recipient;
        // Message to send.
        std::string message;
        // Delivery deadline.
        double deadline;
      };

      //! Parent task.
      Tasks::Task* m_task;
      //! Timer for RSSI Querry
      DUNE::Time::Counter<double> m_rssi_querry_timer;
      //! Timer for Network Check
      DUNE::Time::Counter<double> m_ntwk_querry_timer;
      //! IMEI number of the modem
      std::string m_IMEI;
      //! IMSI number of the SIM card
      std::string m_IMSI;
      //! Phone number of the SIM card
      std::string m_phone_number;
      //! Current State of Modem
      uint8_t m_modem_state = INITIAL_STATE;
      //! Signal Strength
      double m_rssi;
      //! SMS queue.
      std::priority_queue<SmsRequest> m_queue;
      //! SMS timeout
      double m_sms_tout;
      //! Ping Value
      int m_ping;

      TobyL2(Tasks::Task* task , SerialPort* uart):
      HayesModem(task, uart),
      m_task(task)
      {
        sendReset();
        Time::Delay::wait(2.0);
        setLineTrim(true);
        setReadMode(READ_MODE_LINE);
        setTimeout(7.0);
        flushInput();
        start();
        sendInitialization();
      }

      ~TobyL2()
      {

      }

      void
      initTobyL2(const std::string apn , const std::string pin)
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
        setMessageFormat(1);
        //! Remore from Airplane Mode
        setAirplaneMode(false);
      }

      void
      updateTobyL2()
      {
        static uint8_t ping_err = 0;

        if (m_rssi_querry_timer.overflow())
        {
          if (m_modem_state >= NETWORK_REGISTRATION_DONE )
          {
            m_rssi = getRSSI();
            m_task->inf("Current Signal Strength %.2f%% " , m_rssi);
            //! Send RSSI here
          }
          m_rssi_querry_timer.reset();
        }


        if (m_ntwk_querry_timer.overflow())
        {
          if (m_modem_state > NETWORK_REGISTRATION_DONE)
          {
            checkMessages();
            processSMSQueue();
          }

          switch(m_modem_state)
          {
            case INITIAL_STATE:
            {
              if (checkSIMStatus() == 1)
              {
                m_modem_state++;
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
              uint8_t pdp = 0 , state = 0;
              m_task->inf("Radio Access Technology Type %d " , rat_type);

              if (rat_type > 0 && rat_type < 7 && !checkPDPContext(&pdp , &state))
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
                //! Connected to LTE network Do nothing Modem auto connents to internet
                m_modem_state++;
              }
              break;
            }

            case PDP_CONTEXT_ATTACHED:
            {
              //! Check PDP Context
              uint8_t pdp = 0 , state = 0;
              bool status =  checkPDPContext(&pdp , &state);
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
              uint8_t pdp = 0 , state = 0;
              bool status =  checkPDPContext(&pdp , &state);
              if (status == false)
              {
                m_modem_state = INITIAL_STATE;
              }
              else
              {
                m_ping = pingRemote("www.google.com");
                //! PSD is not setup
                if (m_ping == -2)
                {
                  setupPSDProfile();
                }
                //! Error in Ping
                //! Ping Can fail when the connection is bad and the ping time exceeds command timeout
                else if (m_ping == -1)
                {
                  ping_err++;
                  //! Ping Failed 4 times Check Connection status again.
                  if (ping_err > 4)
                  {
                    ping_err = 0;
                    m_modem_state = INITIAL_STATE;
                  }
                }
                else if (m_ping >= 0)
                {
                  ping_err = 0;
                }
              }
              break;
            }
          }
          m_ntwk_querry_timer.reset();
        }
      }

      void
      setSMSTimeout(const double timeout)
      {
        m_sms_tout = timeout;
      }

      void
      setRssiTimer(const double rssi_timer)
      {
        m_rssi_querry_timer.setTop(rssi_timer);
      }

      void
      setNtwkTimer(const double ntwk_timer)
      {
        m_ntwk_querry_timer.setTop(ntwk_timer);
      }

      void
      sendSmsStatus(const SmsRequest* sms_req,IMC::SmsStatus::StatusEnum status,const std::string& info = "")
      {
        IMC::SmsStatus sms_status;
        sms_status.setDestination(sms_req->src_adr);
        sms_status.setDestinationEntity(sms_req->src_eid);
        sms_status.req_id = sms_req->req_id;
        sms_status.info   = info;
        sms_status.status = status;
        m_task->dispatch(sms_status);
      }


    private:
      void
      sendSMS(const std::string& number, const std::string& msg, double timeout)
      {
        uint8_t bfr[16];

        Time::Counter<double> timer(timeout);

        try
        {
          setReadMode(HayesModem::READ_MODE_RAW);
          sendAT(String::str("+CMGS=\"%s\"", number.c_str()));
          readRaw(timer, bfr, 4);
          setReadMode(HayesModem::READ_MODE_LINE);

          if (std::memcmp(bfr, c_sms_prompt, c_sms_prompt_size) != 0)
            throw Hardware::UnexpectedReply();

          std::string data = msg;
          data.push_back(c_sms_term);
          sendRaw((uint8_t*)&data[0], data.size());
        }
        catch (...)
        {
          setReadMode(HayesModem::READ_MODE_LINE);
          throw;
        }

        std::string reply = readLine(timer);
        if (reply == "ERROR")
        {
          throw std::runtime_error(DTR("unknown error"));
        }
        else if (String::startsWith(reply, "+CMGS:"))
        {
          setBusy(true);
        }
        else if (String::startsWith(reply, "+CMS ERROR:"))
        {
          int code = -1;
          std::sscanf(reply.c_str(), "+CMS ERROR: %d", &code);
          throw std::runtime_error(String::str(DTR("SMS transmission failed with error code %d"), code));
        }
        else
        {
          throw UnexpectedReply();
        }

        expectOK();
      }

      void
      checkMessages(void)
      {
        IMC::TextMessage sms;
        std::string location;
        unsigned read_count = 0;
        bool text_mode = true;
        sendAT("+CMGL=\"ALL\"");

        //! Read all messages.
        while (readSMS(location, sms.origin, sms.text,text_mode))
        {
          if ((location == "\"REC UNREAD\"") || (location == "\"REC READ\""))
          {
            ++read_count;
            if(text_mode)
            {
              m_task->inf("Recieved sms from %s , Message %s " , sms.origin.c_str() , sms.text.c_str() );
              m_task->dispatch(sms);
            }
          }
        }
        //! Remove read messages.
        if (read_count > 0)
        {
          sendAT("+CMGD=0,3");
          expectOK();
        }
      }

      bool
      readSMS(std::string& location, std::string& origin, std::string& text, bool& text_mode)
      {
        std::string header = readLine();
        if (header == "OK")
          return false;

        if (!String::startsWith(header, "+CMGL:"))
          throw Hardware::UnexpectedReply();

        std::vector<std::string> parts;
        String::split(header, ",", parts);
        if (parts.size() != 6)
        {
          if (parts.size() >= 2)
          {
            location = parts[1];
            return true;
          }

          throw Hardware::UnexpectedReply();
        }

        if ((parts[2] != "\"\"") && (parts[2].size() <= 2))
          throw Hardware::UnexpectedReply();

        location = parts[1];
        origin = std::string(parts[2], 1, parts[2].size() - 2);
        std::string incoming_data = readLine();

        if(Algorithms::Base64::validBase64(incoming_data))
        {
          text_mode = false;
          Utils::ByteBuffer bfr;
          std::string decoded = Algorithms::Base64::decode(incoming_data);
          std::copy(decoded.begin(),decoded.end(),bfr.getBuffer());
          uint16_t length = decoded.size();
          try
          {
            IMC::Message* msg_d = IMC::Packet::deserialize(bfr.getBuffer(), length);
            m_task->inf(DTR("received IMC message of type %s via SMS"),msg_d->getName());
            m_task->dispatch(msg_d);
          }
          catch(...) //InvalidSync || InvalidMessageId || InvalidCrc
          {
            m_task->war(DTR("Parsing unrecognized Base64 message as text"));
            text.assign(incoming_data);
            text_mode = true;
            return true;
          }
        }
        else
        {
          text.assign(incoming_data);
          text_mode = true;
        }
        return true;
      }

      void
      setMessageFormat(unsigned value)
      {
        sendAT(String::str("+CMGF=%u", value));
        expectOK();
      }

      void
      processSMSQueue(void)
      {
        if (m_queue.empty())
        {
          return;
        }

        SmsRequest sms_req = m_queue.top();
        m_queue.pop();

        // Message is too old, discard it.
        if (Time::Clock::getSinceEpoch() >= sms_req.deadline)
        {
          sendSmsStatus(&sms_req,IMC::SmsStatus::SMSSTAT_INPUT_FAILURE,DTR("SMS timeout"));
          m_task->war(DTR("discarded expired SMS to recipient %s"), sms_req.destination.c_str());
          return;
        }

        try
        {
          sendSMS(sms_req.destination, sms_req.sms_text, m_sms_tout);
          //SMS successfully sent, otherwise driver throws error
          sendSmsStatus(&sms_req,IMC::SmsStatus::SMSSTAT_SENT);
        }
        catch (...)
        {
          m_queue.push(sms_req);
          sendSmsStatus(&sms_req,IMC::SmsStatus::SMSSTAT_ERROR,
                        DTR("Error sending message over GSM modem"));
          m_task->inf(DTR("Error sending SMS to recipient %s"),sms_req.destination.c_str());
        }
      }

      int
      pingRemote(std::string remote)
      {
        int round_trip_time = -1;
        sendAT("+UPING=\""+remote+"\",1,32,5000,255");
        std::string line = readLine();
        if (line == "OK")
        {
          line = readLine();
          //+UUPING: 1,32,\"www.google.com\","172.217.23.100",53,260
          if (line.find("+UUPING:") != std::string::npos)
          {
            std::vector<std::string> tokens;
            String::split(line, ",", tokens);
            if (tokens.size() > 4)
            {
              m_task->inf("Ping Value %s " , tokens[5].c_str());
              std::istringstream iss (tokens[5]);
              iss >> round_trip_time;
            }
          }
          else if (line.find("+UUPINGER: 17") != std::string::npos)
          {
            //! PSD not setup
            m_task->err("Ping Error");
            return -2;
          }
        }
        return round_trip_time;
      }

      void
      activatePDPContext()
      {
        sendAT("+CGACT=1,1");
        expectOK();
      }

      bool
      checkPDPContext(uint8_t* pdp_context , uint8_t* pdp_state)
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
          if ((std::sscanf(arr[i].c_str(), "+CGACT: %d,%d", &cid, &status) == 2 ) && status > 0)
          {
            *pdp_context = cid;
            *pdp_state = status;
            //! Atleast one PDP context is active
            return true;
          }
        }
        return false;
      }

      void
      setupPSDProfile()
      {
        uint8_t pdp , state;
        std::string line;
        if (checkPDPContext(&pdp , &state) && state > 0)
        {
          flushInput();
          //! Map PSD profile to which ever CGACT is active
          std::stringstream at_command;
          at_command << "+UPSD=0,100," << (unsigned int)pdp;
          sendAT(at_command.str());
          line = readLine();
          //! Set PDP Type IPv4
          sendAT("+UPSD=0,0,0");
          line = readLine();
          //! Activate Internal PSD
          sendAT("+UPSDA=0,3");
          line = readLine();
          if (line == "OK")
          {
            line = readLine();
          }
        }
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
        int number;
        std::string line = readValue("+COPS?");
        if (line.find("+COPS") != std::string::npos)
        {
          std::vector<std::string> tokens;
          String::split(line, ",", tokens);
          std::istringstream iss (tokens[3]);
          iss >> number;
          return number;
        }
        return -1;
      }

      int
      checkNetworkRegistration()
      {
        int n = -1 , stat = -1;
        std::string line = readValue("+CREG?");
        if (std::sscanf(line.c_str(), "+CREG: %d,%d", &n, &stat) == 2)
        {
          return stat;
        }
        return -1;
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

      double
      getRSSI()
      {
        int rssi = -1;
        int ber = 0;
        std::string line = readValue("+CSQ");
        if (std::sscanf(line.c_str(), "+CSQ: %d,%d", &rssi, &ber) == 2)
        {
          return convertRSSI(rssi);
        }
        return -1;
      }

      //! This needs to be fixed.
      double
      convertRSSI(int rssi)
      {
        double cvt = -1.0f;
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