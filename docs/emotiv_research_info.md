## The latency of EMOTIV data streams
Latency is the delay between a brain event and the time its effect is seen on the receiving device.

For EPOC X/+ and Insight, there is a 60ms delay in the digital filters in the headset firmware, plus a few more milliseconds of wireless transmission and processing delays in Cortex SDK. We account for those delays in Cortex when we add markers to data, but real-time EEG data passed through the API or displayed using EmotivPRO are shown about 200ms late to allow time for asynchronous API markers to be added correctly to the data stream.

For BCI the above 200ms output delay is not present - the data is processed in our detections as it arrives. The update rate for each detection and the length of the trailing data sample determines the latency in that case.

Performance Metrics are updated twice per second. Mental Commands are updated 4 times per sec. For PMs and Mental Commands, the additional latency comes from the time to detect a change in state.

We look at the last 1 sec of data for Mental Commands. The detection takes up to 250ms to see a significant change in the trailing 1-sec sample.

Typically the Performance Metrics take 0.5-1 seconds to respond to significant changes.


## Everything you need to know about reference sensors of EMOTIV hardware
EPOC+ and EPOC X have two options for positioning CMS (common mode sensor) either at P3 location (slightly left of centre) or on the left mastoid location. DRL (common mode cancellation sensor) can be placed symmetrically on the right side. INSIGHT places both CMS and DRL on the left mastoid location. EPOC Flex (all models) allow completely arbitrary placement of both CMS and DRL reference sensors. EPOC Flex uses the same referencing circuit and strategy as EPOC X. EPOC Flex allows you to position the references anywhere you want them. Ear clips are available for Emotiv EPOC Flex Gel.


Where are the reference sensors? Do you have an impedance check for the hardware? Our electronics use CMS/P3 (left side) as the electrical reference point and DRL/P4 (right side) as the noise cancellation electrode. We use a CMS/DRL common mode cancellation circuit which includes injection of a small high-frequency signal into DRL. We measure the amplitude of that signal at each sensor location to determine the conductivity in real-time, fed back to the user through the contact quality map, with black/red/orange/green indicators at each location. M2/CMS2 and M1/DRL2 are alternative reference sensors. The cover rubbing should go to M2/CSM2 and M1/DRL2.


We use CQ (Contact Quality) to evaluate the impedance and have CQ display in our software. It’s a visual representation of the current contact quality of the individual headset sensors. You can observe each sensor’s status in real-time to adjust sensors to optimize contact quality. The color-coding is Green (good), Orange (moderate), Red (poor), Black (very poor). You can refer to this for more information.


We have also updated our application to have the EEG quality (EQ) that can help to determine the quality of the signal based on multiple metrics. Each of these metrics is important in assessing whether the recording data accurately captures the underlying brain signal. More information about EQ can be found here.


## Can I measure Delta waves (0-4Hz) with Emotiv hardware?
Delta waves (0-4Hz) are hard to measure accurately for several reasons.

Typical ocular artefacts, movement artefacts, heart rate, respiration, blinks and body potential drift also have strong components at low frequencies. Sleep studies measure delta frequency pretty well because the subject barely moves, is usually connected to a ground lead and has their eyes closed.
Accurate measurement of any frequency component with FFT relies on having at least 10 cycles of the lowest frequency of interest in the sample, and assumes it is “stationary”. For 0.5Hz this is at least a 20 sec sample. FFT will round or fill to a power of 2, so 16 sec is a little short for 0.5Hz, 32sec is ideal…
Because FFT needs many cycles to accurately estimate amplitude and phase at a frequency, low frequencies are poorly estimated - and the accuracy decreases as the frequency goes down. The power at frequency f is overestimated by a factor related to 1/f.
We don’t output delta wave power because it’s misleading. If you need it, you will need to apply corrections and maybe a different method than FFT, after removing artefacts