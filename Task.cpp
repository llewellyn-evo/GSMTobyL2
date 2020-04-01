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
      float rssi_querry_per;
      //! Network connection querry period
      float nwk_querry_per;
      //! GSM Pin.
      std::string pin;
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
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("/dev/ttyACM0")
        .description("Serial port device used to communicate with Toby L2");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("115200")
        .description("Serial port baud rate");

        param("Power Channel - Names", m_args.pwr_channel_name)
        .defaultValue("GSM")
        .description("Device's power channels");

        param("RSSI Querry Periodicity", m_args.rssi_querry_per)
        .defaultValue("10")
        .units(Units::Second)
        .description("Periodicity of RSSI reports");

        param("Network Querry Periodicity", m_args.nwk_querry_per)
        .defaultValue("10")
        .units(Units::Second)
        .description("Periodicity of RSSI reports");

        param("PIN", m_args.pin)
        .defaultValue("")
        .description("PIN Code");

        param("APN", m_args.apn_name)
        .defaultValue("web.vodafone.de")
        .description("APN Code");
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
        dispatch(pcc);
        //! Wait here for 5 seconds to kernel to detect and bring the device UP
        Delay::waitNsec(5000000000);
        //! Create Handle for Serial Port to configure GSM Modem

        m_uart = new SerialPort(m_args.uart_dev, m_args.uart_baud);
        if (!m_modem)
        {
          m_modem = new TobyL2(this , m_uart);
          m_modem->initTobyL2(m_args.apn_name ,  m_args.pin , m_args.rssi_querry_per ,  m_args.nwk_querry_per);
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
        Memory::clear(m_modem);
        Memory::clear(m_uart);
      }

      //! Main loop.
      void
      onMain(void)
      {

        while (!stopping())
        {
          m_modem->updateTobyL2();
          waitForMessages(0.05);
        }
      }
    };
  }
}

DUNE_TASK
