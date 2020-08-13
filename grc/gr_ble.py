#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: ble_receiver
# GNU Radio version: 3.8.1.0

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
from gnuradio import zeromq


class gr_ble(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "ble_receiver")

        ##################################################
        # Variables
        ##################################################
        self.transition_width = transition_width = 300e3
        self.sample_rate = sample_rate = 4e6
        self.cutoff_freq = cutoff_freq = 850e3
        self.ble_channel_spacing = ble_channel_spacing = 2e6
        self.ble_channel = ble_channel = 0
        self.ble_base_freq = ble_base_freq = int(2402e6)
        self.squelch_threshold = squelch_threshold = -80
        self.lowpass_filter = lowpass_filter = firdes.low_pass(1, sample_rate, cutoff_freq, transition_width, firdes.WIN_HAMMING, 6.76)
        self.gmsk_sps = gmsk_sps = 4
        self.gmsk_omega_limit = gmsk_omega_limit = 0.035
        self.gmsk_mu = gmsk_mu = 0.5
        self.gmsk_gain_mu = gmsk_gain_mu = 0.7
        self.freq_offset = freq_offset = 1e6
        self.freq = freq = (ble_base_freq+(ble_channel_spacing*ble_channel))

        ##################################################
        # Blocks
        ##################################################
        self.zeromq_push_sink_0 = zeromq.push_sink(gr.sizeof_char, 1, 'tcp://127.0.0.1:5557', 100, False, -1)
        self.uhd_usrp_source_0 = uhd.usrp_source(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.uhd_usrp_source_0.set_center_freq(freq-freq_offset, 0)
        self.uhd_usrp_source_0.set_rx_agc(True, 0)
        self.uhd_usrp_source_0.set_antenna('RX2', 0)
        self.uhd_usrp_source_0.set_samp_rate(sample_rate)
        self.uhd_usrp_source_0.set_time_now(uhd.time_spec(time.time()), uhd.ALL_MBOARDS)
        self.freq_xlating_fir_filter_xxx_0 = filter.freq_xlating_fir_filter_ccc(1, lowpass_filter, 1e6, sample_rate)
        self.digital_gmsk_demod_0 = digital.gmsk_demod(
            samples_per_symbol=gmsk_sps,
            gain_mu=gmsk_gain_mu,
            mu=gmsk_mu,
            omega_relative_limit=gmsk_omega_limit,
            freq_error=0.0,
            verbose=False,log=False)
        self.blocks_unpacked_to_packed_xx_0 = blocks.unpacked_to_packed_bb(1, gr.GR_LSB_FIRST)
        self.analog_simple_squelch_cc_1 = analog.simple_squelch_cc(squelch_threshold, 0.1)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_simple_squelch_cc_1, 0), (self.freq_xlating_fir_filter_xxx_0, 0))
        self.connect((self.blocks_unpacked_to_packed_xx_0, 0), (self.zeromq_push_sink_0, 0))
        self.connect((self.digital_gmsk_demod_0, 0), (self.blocks_unpacked_to_packed_xx_0, 0))
        self.connect((self.freq_xlating_fir_filter_xxx_0, 0), (self.digital_gmsk_demod_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.analog_simple_squelch_cc_1, 0))


    def get_transition_width(self):
        return self.transition_width

    def set_transition_width(self, transition_width):
        self.transition_width = transition_width
        self.set_lowpass_filter(firdes.low_pass(1, self.sample_rate, self.cutoff_freq, self.transition_width, firdes.WIN_HAMMING, 6.76))

    def get_sample_rate(self):
        return self.sample_rate

    def set_sample_rate(self, sample_rate):
        self.sample_rate = sample_rate
        self.set_lowpass_filter(firdes.low_pass(1, self.sample_rate, self.cutoff_freq, self.transition_width, firdes.WIN_HAMMING, 6.76))
        self.uhd_usrp_source_0.set_samp_rate(self.sample_rate)

    def get_cutoff_freq(self):
        return self.cutoff_freq

    def set_cutoff_freq(self, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        self.set_lowpass_filter(firdes.low_pass(1, self.sample_rate, self.cutoff_freq, self.transition_width, firdes.WIN_HAMMING, 6.76))

    def get_ble_channel_spacing(self):
        return self.ble_channel_spacing

    def set_ble_channel_spacing(self, ble_channel_spacing):
        self.ble_channel_spacing = ble_channel_spacing
        self.set_freq((self.ble_base_freq+(self.ble_channel_spacing*self.ble_channel)))

    def get_ble_channel(self):
        return self.ble_channel

    def set_ble_channel(self, ble_channel):
        self.ble_channel = ble_channel
        self.set_freq((self.ble_base_freq+(self.ble_channel_spacing*self.ble_channel)))

    def get_ble_base_freq(self):
        return self.ble_base_freq

    def set_ble_base_freq(self, ble_base_freq):
        self.ble_base_freq = ble_base_freq
        self.set_freq((self.ble_base_freq+(self.ble_channel_spacing*self.ble_channel)))

    def get_squelch_threshold(self):
        return self.squelch_threshold

    def set_squelch_threshold(self, squelch_threshold):
        self.squelch_threshold = squelch_threshold
        self.analog_simple_squelch_cc_1.set_threshold(self.squelch_threshold)

    def get_lowpass_filter(self):
        return self.lowpass_filter

    def set_lowpass_filter(self, lowpass_filter):
        self.lowpass_filter = lowpass_filter
        self.freq_xlating_fir_filter_xxx_0.set_taps(self.lowpass_filter)

    def get_gmsk_sps(self):
        return self.gmsk_sps

    def set_gmsk_sps(self, gmsk_sps):
        self.gmsk_sps = gmsk_sps

    def get_gmsk_omega_limit(self):
        return self.gmsk_omega_limit

    def set_gmsk_omega_limit(self, gmsk_omega_limit):
        self.gmsk_omega_limit = gmsk_omega_limit

    def get_gmsk_mu(self):
        return self.gmsk_mu

    def set_gmsk_mu(self, gmsk_mu):
        self.gmsk_mu = gmsk_mu

    def get_gmsk_gain_mu(self):
        return self.gmsk_gain_mu

    def set_gmsk_gain_mu(self, gmsk_gain_mu):
        self.gmsk_gain_mu = gmsk_gain_mu

    def get_freq_offset(self):
        return self.freq_offset

    def set_freq_offset(self, freq_offset):
        self.freq_offset = freq_offset
        self.uhd_usrp_source_0.set_center_freq(self.freq-self.freq_offset, 0)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_source_0.set_center_freq(self.freq-self.freq_offset, 0)





def main(top_block_cls=gr_ble, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()

    try:
        input('Press Enter to quit: ')
    except EOFError:
        pass
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
