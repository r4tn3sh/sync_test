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
        std::cout << "PN sequence detection started" << std::endl;
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


            sem_post(&m_pause); // Flags the end of this loop and wakes up any other threads waiting on this semaphore
                                // i.e. a call to the pause() function in the main thread.

            if(pk_index < PKTLEN)
            {
                m_callback(1);
                std::cout<< "Signal found at " << pk_index << std::endl;
                break;
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time-begin_time).count();
            if(duration > 30000000)
            {
                m_callback(0);
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
        else
        {
            // std::cout << "File opened for correlation "<< samples.size() << std::endl;
        }
        char pnseq_c[ULSEQLEN];
        double pnseq[ULSEQLEN];
        pnfile.read(pnseq_c, ULSEQLEN);

        for (int i=0; i<ULSEQLEN; i++)
        {
            pnseq[i] = 2*((double)(pnseq_c[i]))-1;
            // std::cout << pnseq[i] << " " ;
        }

        std::complex<double> temp_mul;
        std::complex<double> temp_mean;
        double temp_norm_v;
        double corr_coeff = 0;
        double sqr_sum = 0;
        double numr;
        double denm;
        unsigned int N = ULSEQLEN;

        double pn_mean=0;
        for (int j=0; j<N; j++)
        {
            pn_mean += pnseq[j];
        }
        pn_mean/=N;
        double test_thresh = 0.8;
        // std::cout << "PN mean : " << pn_mean << std::endl;
        for (int i=0; i<PKTLEN; i++)
        {
            temp_mul = (0, 0);
            temp_mean = (0, 0);
            sqr_sum = 0;
            for (int j=0; j<N; j++)
            {
                // temp_mul.real() += samples[i+j].real() * pnseq[j];
                // temp_mul.imag() += samples[i+j].imag() * pnseq[j];
                temp_mul += samples[i+j] * pnseq[j];
                sqr_sum += pow(abs(samples[i+j]),2);
                temp_mean += samples[i+j];
            }
            // std::cout << "Sample sum : " << temp_mean << std::endl;
            temp_mean/=N;
            temp_norm_v = 0;
            // for (int j=0; j<ULSEQLEN; j++)
            // {
            //     // temp_norm_v += abs((samples[i+j]-temp_mean)*conj(samples[i+j]-temp_mean));
            //     std::cout <<  " " << samples.at(i+j) <<" " << pow(abs(samples.at(i+j)),2) << std::endl;
            //     // std::cout <<  " " << samples.at(i+j) <<" " << pow(abs(samples.at(i+j)-temp_mean),2) << std::endl;
            //     // std::cout << " " << temp_norm_v << std::endl;
            // }
            // corr_coeff = abs(temp_mul-temp_mean)/(sqrt(temp_norm_v*ULSEQLEN));
            // corr_coeff = abs(temp_mul)/(sqrt(temp_norm_v*ULSEQLEN));
            numr = abs(temp_mul)-N*abs(temp_mean*pn_mean);
            denm = sqrt(sqr_sum-N*pow(abs(temp_mean),2))*sqrt(N);
            corr_coeff = numr/denm;
            
            if(corr_coeff<-test_thresh || corr_coeff>test_thresh)
                std::cout << "Correlation coefficient : " << corr_coeff << " " << i <<  std::endl;
            if(corr_coeff<-1 || corr_coeff>1)
            {
                std::cout << numr << "/" << denm << std::endl;
                std::cout << "Correlation coefficient : " << corr_coeff << " " << abs(temp_mul) << " " << sqr_sum << " " << pow(abs(temp_mean),2) << " " <<(sqr_sum-N*pow(abs(temp_mean),2))*N <<  std::endl;
            }
            
            if(corr_coeff>COEFFTHRESH)
            {
                for (int j=0; j<N; j++)
                {
                    std::cout <<  " " << samples.at(i+j) <<" " << pnseq[j] << std::endl;
                }
                std::cout << "Correlation coefficient above threshold. " << corr_coeff << std::endl;
                std::cout <<  temp_mul << " " << temp_mean << " " << temp_norm_v << " " << std::endl;
                return(i);
            }
        }
        return(PKTLEN);
    }

    /*!
     *  Uses an internal semaphore to block the execution of the ul_receiver loop code effectively pausing
     *  the ul_receiver until the semaphore is posted to (cleared) by the ul_receiver::resume() function.
     */
    void ul_receiver::pause()
    {
        std::cout << "Pausing the receiver" << std::endl;
        sem_wait(&m_pause);
    }

    /*!
     *  This function posts to (clears) the internal semaphore that is blocking the ul_receiver loop code execution
     *  due to a previous call to the ul_receiver::pause() function, thus allowing the main ul_receiver loop to begin
     *  executing again.
     */
    void ul_receiver::resume()
    {
        std::cout << "Resuming the receiver" << std::endl;
        sem_post(&m_pause);
    }
}
