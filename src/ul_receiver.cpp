/*! \file ul_receiver.h
 *  \brief C++ file for the ul_receiver class.
 *
 *  The ul_receiver class is the public interface for the fun_ofdm ul_receiver.
 *  This is the easiest way to start receiving 802.11a OFDM frames out of the box.
 */

#include "ul_receiver.h"
#include <math.h> 
#include <chrono>
#include <iostream>
#include <fstream>

namespace fun
{

    /*!
     * This constructor shows exactly what parameters need to be set for the ul_receiver.
     */
    ul_receiver::ul_receiver(void (*callback)(int stp), double freq, double samp_rate, double rx_gain, std::string device_addr) :
        ul_receiver(callback, usrp_params(freq, samp_rate, 20, rx_gain, 1.0, device_addr))
    {
    }

    /*!
     * This constructor is for those who feel more comfortable using the usrp_params struct.
     */
    ul_receiver::ul_receiver(void (*callback)(int stp), usrp_params params) :
        m_usrp(params),
        m_samples(NUM_RX_SAMPLES),
        m_callback(callback)
    {
        sem_init(&m_pause, 0, 1); //Initial value is 1 so that the ul_receiver_chain_loop() will begin executing immediately
        m_rec_thread = std::thread(&ul_receiver::seq_detector_loop, this); //Initialize the main ul_receiver thread
        // m_rec_thread = std::thread(&ul_receiver::ul_receiver_chain_loop, this); //Initialize the main ul_receiver thread
    }
    /*!
     *  This function loops forever (unless it is paused) pulling samples from the USRP and passing them through the
     *  ul_receiver chain. It then passes any successfully decoded packets to the callback function for the user
     *  to process further. This function can be paused by the user by calling the ul_receiver::pause() function,
     *  presumably so that the user can transmit packets over the air using the transmitter. Once the user is finished
     *  transmitting he/she can resume the ul_receiver by called the ul_receiver::resume() function. These two functions use
     *  an internal semaphore to block the ul_receiver code execution while in the paused state.
     */
    void ul_receiver::seq_detector_loop()
    {
        auto begin_time = std::chrono::high_resolution_clock::now();
        // auto end_time = 0;
        // auto duration = 0;
        while(1)
        {
            sem_wait(&m_pause); // Block if the ul_receiver is paused

            m_usrp.get_samples(NUM_RX_SAMPLES, m_samples);

            // std::vector<std::vector<unsigned char> > packets =
            //         m_rec_chain.process_samples(m_samples);
            int pk_index = correlate_ulseq(m_samples);

            int stp = 0;
            m_callback(stp);

            sem_post(&m_pause); // Flags the end of this loop and wakes up any other threads waiting on this semaphore
                                // i.e. a call to the pause() function in the main thread.

            if(pk_index < PKTLEN)
            {
                std::cout<< "Signal found" << std::endl;
                break;
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time-begin_time).count();
            if(duration > 2000000)
            {
                std::cout<< "Timeout occured" << std::endl;
                break;
            }
        }
    }

    /*!
     * \Correlate and find signal and return the index of the peak
     */
    int ul_receiver::correlate_ulseq(std::vector<std::complex<double> > samples)
    {
        std::ifstream pnfile; 
        pnfile.open("pnseq.dat", std::ios::in | std::ios::binary);
        if(!pnfile) 
        {
            std::cout << "Cannot open file.\n";
            return(-1);
        }
        char pnseq_c[ULSEQLEN];
        double pnseq[ULSEQLEN];
        pnfile.read(pnseq_c, ULSEQLEN);

        std::complex<double> temp_mul;
        std::complex<double> temp_mean;
        double temp_norm_v;
        double corr_coeff;
        for (int i=0; i<PKTLEN; i++)
        {
            temp_mul = (0, 0);
            temp_mean = (0, 0);
            for (int j=0; j<ULSEQLEN; j++)
            {
                // temp_mul.real() += samples[i+j].real() * pnseq[j];
                // temp_mul.imag() += samples[i+j].imag() * pnseq[j];
                temp_mul += samples[i+j] * pnseq[j];
                temp_mean += samples[j];
            }
            temp_mean/=ULSEQLEN;
            temp_norm_v = 0;
            for (int j=0; j<ULSEQLEN; j++)
            {
                temp_norm_v += abs(samples[i+j]-temp_mean)*abs(samples[i+j]-temp_mean);
            }
            corr_coeff = abs(temp_mul)/sqrt(temp_norm_v);
            
            if(corr_coeff>COEFFTHRESH)
                return(i);
        }
        return(PKTLEN);
    }

    /*!
     *  Uses an internal semaphore to block the execution of the ul_receiver loop code effectively pausing
     *  the ul_receiver until the semaphore is posted to (cleared) by the ul_receiver::resume() function.
     */
    void ul_receiver::pause()
    {
        sem_wait(&m_pause);
    }

    /*!
     *  This function posts to (clears) the internal semaphore that is blocking the ul_receiver loop code execution
     *  due to a previous call to the ul_receiver::pause() function, thus allowing the main ul_receiver loop to begin
     *  executing again.
     */
    void ul_receiver::resume()
    {
        sem_post(&m_pause);
    }
}
