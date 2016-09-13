/*! \file ul_receiver.h
 *  \brief Header file for the ul_receiver class.
 *
 *  The ul_receiver class is the public interface for the fun_ofdm ul_receiver.
 *  This is the easiest way to start receiving 802.11a OFDM frames out of the box.
 */

#ifndef UL_RECEIVER_H
#define UL_RECEIVER_H

#include <semaphore.h>
#include <vector>
#include <thread>
// #include "ul_receiver_chain.h"
#include "usrp.h"


#define PKTLEN 640
#define ULSEQLEN 160
#define NUM_RX_SAMPLES 800
#define COEFFTHRESH 0.8

namespace fun
{

    /*!
     * \brief The ul_receiver class is the public interface for the fun_ofdm ul_receiver.
     *  This is the easiest way to start receiving 802.11a OFDM frames out of the box.
     *
     *  Usage: To receive packets simply create a ul_receiver object and pass it a callback
     *  function that takes a std::vector<std::vector<unsigned char> > as an input parameter.
     *  The ul_receiver object then automatically creates a separate thread that pulls samples
     *  from the USRP and processes them with the receive chain. The received packets (if any)
     *  are then passed into the callback function where the user is able to process them further.
     *
     *  If at any time the user wishes to pause the ul_receiver (i.e. so that the user can transmit
     *  some packets) the user simply needs to call the ul_receiver::pause() function on the ul_receiver
     *  object. Similarly, the ul_receiver::resume() function can then be used to begin receiving again
     *  after a pause.
     */
    class ul_receiver
    {
    public:

        /*!
         * \brief Constructor for the ul_receiver with raw parameters
         * \param callback Function pointer to the callback function where received packets are passed
         * \param freq [Optional] Center frequency
         * \param samp_rate [Optional] Sample Rate
         * \param rx_gain [Optional] Receive Gain
         * \param device_addr [Optional] IP address of USRP device
         *
         *  Defaults to:
         *  - center freq -> 5.72e9 (5.72 GHz)
         *  - sample rate -> 5e6 (5 MHz)
         *  - rx gain -> 20
         *  - device ip address -> "" (empty string will default to letting the UHD api
         *    automatically find an available USRP)
         *  - *Note:
         *    + tx_gain -> 20 even though it is irrelevant for the ul_receiver
         *    + amp -> 1.0 even though it is irrelevant for the ul_receiver
         */
        ul_receiver(void(*callback)(int stp), double freq = 5.72e9, double samp_rate = 5e6, double rx_gain = 20, std::string device_addr = "");

        /*!
         * \brief Constructor for the ul_receiver that uses the usrp_params struct
         * \param callback Function pointer to the callback function where received packets are passed
         * \param params [Optional] The usrp parameters you want to use for this ul_receiver.
         *
         *  Defaults to:
         *  - center freq -> 5.72e9 (5.72 GHz)
         *  - sample rate -> 5e6 (5 MHz)
         *  - tx gain -> 20
         *  - rx gain -> 20 (although this is irrelevant for the transmitter)
         *  - device ip address -> "" (empty string will default to letting the UHD api
         *    automatically find an available USRP)
         */
        ul_receiver(void(*callback)(int stp), usrp_params params = usrp_params());

        /*!
         * \brief Pauses the ul_receiver thread.
         */
        void pause();

        /*!
         * \brief Resumes the ul_receiver thread after it has been paused.
         */
        void resume();

        /*!
         * \Correlate and find signal and return the index of the peak
         */
        int correlate_ulseq(std::vector<std::complex<double> > samples);

    private:

        void ul_receiver_chain_loop(); //!< Infinite while loop where samples are received from USRP and processed by the ul_receiver_chain

        void (*m_callback)(int stp); //!< Callback function pointer

        usrp m_usrp; //!< The usrp object used to ul_receiver frames over the air

        void seq_detector_loop();
        // ul_receiver_chain m_rec_chain; //!< The ul_receiver chain object used to detect & decode incoming frames

        std::vector<std::complex<double> > m_samples; //!< Vector to hold the raw samples received from the USRP and passed into the ul_receiver_chain

        std::thread m_rec_thread; //!< The thread that the ul_receiver chain runs in

        sem_t m_pause; //!< Semaphore used to pause the ul_receiver thread




    };


}

#endif // UL_RECEIVER_H
