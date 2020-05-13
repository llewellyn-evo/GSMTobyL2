//***************************************************************************
// Copyright 2007-2020 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Llewellyn-Fernandes                                              *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include "TobyL2.hpp"

namespace Transports
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author Llewellyn-Fernandes

  struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Power channel Name.
      std::string pwr_channel_name;
      //! APN name to connect to
      std::string apn_name;
      //! RSSI query timer.
      double rssi_querry_per;
      //! Network connection querry period
      double nwk_querry_per;
      //! Network report period
      double nwk_report_per;
      //! GSM Pin.
      std::string pin;
      //! SMS send timeout (s).
      double sms_tout;
      //! start GSM by default flag
      bool start_gsm;
    };

  namespace GSMTobyL2
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Serial port handle.
      SerialPort* m_uart;
      //! Toby L2
      TobyL2* m_modem;
      //! Channel State
      bool m_channel_state = false;
      //! Timer for Network reports
      DUNE::Time::Counter<double> m_ntwk_report_timer;
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_uart(NULL),
        m_modem(NULL)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyACM0")
        .description("Serial port device used to communicate with Toby L2");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        param("Power Channel - Name", m_args.pwr_channel_name)
        .defaultValue("SAT_GSM")
        .description("GSM Device power channel name");

        param("RSSI Querry Periodicity", m_args.rssi_querry_per)
        .defaultValue("10")
        .units(Units::Second)
        .description("Periodicity of RSSI querry");

        param("Network Querry Periodicity", m_args.nwk_querry_per)
        .defaultValue("5")
        .units(Units::Second)
        .description("Periodicity of network status querry");

        param("Network Reports Periodicity", m_args.nwk_report_per)
        .defaultValue("5")
        .units(Units::Second)
        .description("Periodicity of network reports");

        param("SIM-PIN", m_args.pin)
        .defaultValue("")
        .description("SIM card PIN Code");

        param("APN", m_args.apn_name)
        .defaultValue("web.vodafone.de")
        .description("APN Code");

        param("Turn GSM ON", m_args.start_gsm)
        .defaultValue("false")
        .description("Flag to turn GSM ON by Default");

        param("SMS Send Timeout", m_args.sms_tout)
        .defaultValue("60")
        .units(Units::Second)
        .description("Maximum amount of time to wait for SMS send completion");

        bind<IMC::PowerChannelState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (m_modem)
        {
          if (paramChanged(m_args.pin) || paramChanged(m_args.uart_dev) || paramChanged(m_args.uart_baud) || paramChanged(m_args.apn_name))
          {
            throw RestartNeeded(DTR("restarting to change parameters"), 1);
          }
          else if (paramChanged(m_args.rssi_querry_per))
          {
            m_modem->setRssiTimer(m_args.rssi_querry_per);
          }
          else if (paramChanged(m_args.nwk_querry_per))
          {
            m_modem->setNtwkTimer(m_args.nwk_querry_per);
          }
          else if (paramChanged(m_args.sms_tout))
          {
            m_modem->setSMSTimeout(m_args.sms_tout);
          }
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        //! Turn on GSM Channel
        IMC::PowerChannelControl pcc;
        pcc.name = m_args.pwr_channel_name;
        pcc.op = IMC::PowerChannelControl::PCC_OP_TURN_ON;

        while (!m_channel_state && !stopping())
        {
          Time::Delay::wait(2.0);
          if (m_args.start_gsm)
          {
            dispatch(pcc);
          }
          waitForMessages(0.05);
          this->inf("Waiting for channel to be turned ON");
        }
        if (!m_modem && !stopping())
        {
          try
          {
            //! Wait here for 20 seconds to kernel to detect and bring the device UP
            Time::Delay::wait(20.0);
            //! Create Handle for Serial Port to configure GSM Modem
            m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
            m_modem = new TobyL2(this , m_uart);
            m_modem->initTobyL2(m_args.apn_name ,  m_args.pin);
            m_modem->setSMSTimeout(m_args.sms_tout);
            m_modem->setNtwkTimer(m_args.nwk_querry_per);
            m_modem->setRssiTimer(m_args.rssi_querry_per);
            m_ntwk_report_timer.setTop(m_args.nwk_report_per);
            //! Now that its initialized accept SMS send request
            bind<IMC::SmsRequest>(this);
          }
          catch(...)
          {
             throw RestartNeeded(DTR("Restarting.."), 1);
          }
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {

      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_modem)
        {
          m_modem->stopAndJoin();
          delete m_modem;
          m_modem = NULL;
        }
        Memory::clear(m_uart);
      }

      void
      consume(const IMC::PowerChannelState* msg)
      {
        if (msg->name == m_args.pwr_channel_name)
        {
          m_channel_state = (msg->state) ? true:false;
        }
      }

      void
      consume(const IMC::SmsRequest* msg)
      {
        TobyL2::SmsRequest sms_req;
        sms_req.req_id      = msg->req_id;
        sms_req.destination = msg->destination;
        sms_req.sms_text    = msg->sms_text;
        sms_req.src_adr     = msg->getSource();
        sms_req.src_eid     = msg->getSourceEntity();

        if (msg->timeout <= 0)
        {
          m_modem->sendSmsStatus(&sms_req,IMC::SmsStatus::SMSSTAT_INPUT_FAILURE,"SMS timeout cannot be zero");
          inf("%s", DTR("SMS timeout cannot be zero"));
          return;
        }
        if(sms_req.sms_text.length() > 160) //160 characters encoded in 8-bit alphabet per SMS message
        {
          m_modem->sendSmsStatus(&sms_req,IMC::SmsStatus::SMSSTAT_INPUT_FAILURE,"Can only send 160 characters over SMS.");
          inf("%s", DTR("Can only send 160 characters over SMS"));
        return;
        }
        sms_req.deadline = Clock::getSinceEpoch() + msg->timeout;
        m_modem->m_queue.push(sms_req);
        m_modem->sendSmsStatus(&sms_req,IMC::SmsStatus::SMSSTAT_QUEUED,DTR("SMS sent to queue"));
      }

      void
      sendNetworkReports()
      {
        if (m_ntwk_report_timer.overflow())
        {
          //! Dispatch RSSI
          IMC::RSSI rssi;
          rssi.value = m_modem->m_rssi;
          dispatch(rssi);

          //! Dispatch Link Latency
          IMC::LinkLatency link_latency;
          link_latency.value = (m_modem->m_ping) ? (m_modem->m_ping / 1000.0):m_modem->m_ping;
          dispatch(link_latency);

          m_ntwk_report_timer.reset();
        }
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          sendNetworkReports();
          try
          {
            m_modem->updateTobyL2();
          }
          catch(...)
          {
            //! Timeout error. Or GSM modem turned OFF(Serial will dissapear)
            throw RestartNeeded(DTR("Restarting.."), 1);
          }

          waitForMessages(0.05);
        }
      }
    };
  }
}

DUNE_TASK
