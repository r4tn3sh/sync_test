/*! \file transmitter.cpp
 *  \brief C++ file for the transmitter class.
 *
 *  The transmitter class is the public interface for the fun_ofdm transmit chain.
 *  This is the easiest way to start transmitting 802.11a OFDM frames out of the box.
 */

#include "ul_transmitter.h"
#include <iostream>

namespace fun {

    /*!
     *  This constructor shows exactly what parameters need to be set for the transmitter
     */
    ul_transmitter::ul_transmitter(double freq, double samp_rate, double tx_gain, double tx_amp, std::string device_addr) :
        m_usrp(usrp_params(freq, samp_rate, tx_gain, 20, tx_amp, device_addr))
    {
    }

    /*!
     * This construct is for those who feel more comfortable using the usrp_params struct
     */
    ul_transmitter::ul_transmitter(usrp_params params) :
        m_usrp(params)
    {
    }

    /*!
     *  Transmits a single frame, blocking until the frame is sent.
     */
    void ul_transmitter::send_data(std::vector<unsigned char> payload, Rate phy_rate)
    {
        //std::cout << "Start sending data "<< payload.size() << std::endl;
        std::vector<std::complex<double> > samples(payload.size());
        for (int i = 0; i<payload.size(); i++)
        {
            // samples.push_back(0.5, 0.0);
            samples[i] = 0.5*std::complex<double>((2*(double)payload[i])-1, 0.0);
        }
        // std::cout << "Samples ready" << std::endl;
        m_usrp.send_burst_sync(samples);
        // m_usrp.send_burst(samples);
    }

}
