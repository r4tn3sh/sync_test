/*! \file test_tx.cpp
 *  \brief Transmitter test
 *
 *  This file is used to test transmitting OFDM PHY frames over the air.
 */

#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include <iostream>
#include <fstream>
#include "ul_transmitter.h"
#include "ul_receiver.h"

#define PNSEQLEN 2000
#define PKTLEN 640
#define ULSEQLEN 160
using namespace fun;

void callback(int stp);
void test_tx(double freq, double sample_rate, double tx_gain, double amp, Rate phy_rate);
bool set_realtime_priority();
bool get_pnsequence();
int wait_for_signal();

double freq = 2e9;
double sample_rate = 10e6;
double tx_gain = 30;
double rx_gain = 20;
double amp = 0.5;

bool cbflag = false;

char pnseq[PNSEQLEN];
unsigned int pnoffset;
Rate phy_rate = RATE_1_2_BPSK;

int main(int argc, char * argv[]){

    if (argc<2)
    {
        std::cout << "Not enough arguments." << std::endl;
        return(-1);
    }
    else
    {
        pnoffset = strtol(argv[1], NULL, 10);
    }
    set_realtime_priority();

    srand(time(NULL)); //Initialize random seed

    get_pnsequence();
    usrp_params params = usrp_params();
    std::cout << params.device_addr <<std::endl;
    ul_transmitter tx = ul_transmitter(freq, sample_rate, tx_gain, amp, params.device_addr);
    // ul_transmitter tx = ul_transmitter(freq, params);
    // ul_receiver rx = ul_receiver(&callback, freq, sample_rate, rx_gain, amp, params.device_addr);
    ul_receiver rx = ul_receiver(&callback, params);

    // Prepare the packets
    std::vector<unsigned char> ulseq(ULSEQLEN);
    std::vector<unsigned char> packets(PKTLEN,0);

    for(int i = 0; i < ULSEQLEN; i++)
    {
        ulseq[i] = pnseq[pnoffset + i];
        packets[i] = pnseq[pnoffset + i];
    }
    //int waitover = wait_for_signal();

    //////////////////////////////////////////
    std::cout << "Start: wait for signal." << std::endl;
    std::cout << cbflag << std::endl;
    tx.txflagtimefrac = 0.0;
    tx.txflagtimefull = 0.0;
    while(1)
    {
         if (cbflag==true)
         {
             tx.txflagtimefrac = rx.flagtimefrac;
             tx.txflagtimefull = rx.flagtimefull;
             // std::cout << "Break reached. Transmission begins at " << tx.txflagtime  << "::"<< std::endl;
             break;
         }
         else
             std::cout << "" ;
     }
    rx.pause();
    std::cout << "End: wait for signal." << std::endl;
    cbflag = false;
    //////////////////////////////////////////
	
    std::cout << "Start transmit chain..." << std::endl;
    // test_tx(freq, sample_rate, tx_gain, amp, phy_rate);

    //Transmit all the packets
    std::string tx_phy_rate = RateParams(phy_rate).name;
    while(1) // Don't have a very good reason for this
    {
            tx.send_data(packets, phy_rate);
    }
    //////////////////////////////////////////

	return 0;
}

/*!
 * \brief Function for testing the fun_ofdm transmitter
 * \param freq Center Frequency
 * \param sample_rate Sample Rate
 * \param tx_gain Transmitter Gain
 * \param amp Transmit Amplitude
 * \param phy_rate The PHY Rate used for all packets in this tx test
 *
 *  This function transmits 1000 packets. The data in the transmitted packets
 *  is mostly random except for the first, middle, and last 100 bytes which are a
 *  known string. This string is used to verify that the packet was in fact received
 *  along with the IEEE CRC-32 check.
 */
void test_tx(double freq, double sample_rate, double tx_gain, double amp, Rate phy_rate)
{
    // srand(time(NULL)); //Initialize random seed

    // get_pnsequence();
    // usrp_params params = usrp_params();
    // ul_transmitter tx = ul_transmitter(freq, sample_rate, tx_gain, amp);
    // // ul_transmitter tx = ul_transmitter(freq, params);

    // std::vector<unsigned char> ulseq(ULSEQLEN);
    // std::vector<unsigned char> packets(PKTLEN,0);

    // for(int i = 0; i < ULSEQLEN; i++)
    // {
    //     ulseq[i] = pnseq[pnoffset + i];
    //     packets[i] = pnseq[pnoffset + i];
    // }

    // //Transmit all the packets
    // std::string tx_phy_rate = RateParams(phy_rate).name;
    // for(int i = 0; i < 10000; i++)
    // {
    //     tx.send_data(packets, phy_rate);
    // }

}

/*!
 * \Wait for the boundry detected from other transmission
 */
int wait_for_signal()
{
    // std::cout << "Start: wait for signal." << std::endl;
    // std::cout << cbflag << std::endl;
    // usrp_params params = usrp_params();
    // // ul_receiver rx = ul_receiver(&callback, freq, sample_rate, tx_gain, amp);
    // ul_receiver rx = ul_receiver(&callback, params);
    // while(1)
    // {
    //      if (cbflag==true)
    //      {
    //          std::cout << "Break reached." << std::endl;
    //          break;
    //      }
    //      else
    //          std::cout << "*" ;
    //  }
    // rx.pause();
    // std::cout << "End: wait for signal." << std::endl;
    // cbflag = false;
    // return 0;
}

/*!
 * \Read the PN sequence from the file generated by MATLAB
 */
bool get_pnsequence()
{
    std::ifstream pnfile; 
    pnfile.open("pnseq.dat", std::ios::in | std::ios::binary);
    if(!pnfile) 
    {
        std::cout << "Cannot open file.\n";
        return false;
    }
    pnfile.read(pnseq, PNSEQLEN);

    for(int i; i<100; i++)
        std::cout << (int)pnseq[i] << " ";
    std::cout << std::endl;
}

/*!
 * \brief Attempt to set real time priority for thread scheduling
 * \return Whether or not real time priority was succesfully set.
 */
bool set_realtime_priority()
{
    std::cout << "Start: set realtime priority." << std::endl;
    // Get the current thread
    pthread_t this_thread = pthread_self();

    // Set priority to SCHED_FIFO
    struct sched_param params;
    params.sched_priority = sched_get_priority_max(SCHED_RR);
    if (pthread_setschedparam(this_thread, SCHED_RR, &params) != 0)
    {
        std::cout << "Unable to set realtime priority. Did you forget to sudo?" << std::endl;
        return false;
    }

    std::cout << "End: set realtime priority." << std::endl;
    return true;
}





void callback(int stp)
{
    std::cout << "Received a callback : "<< stp << std::endl;
    cbflag = true;
    std::cout << cbflag << std::endl;
}




