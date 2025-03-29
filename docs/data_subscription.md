## Data Subscription
After you opened a session with a headset, you can subscribe to one or more data streams.

Each data steam gives you real time access to data from the headset (EEG, motion...) or data calculated by Cortex (band powers, mental command...)

After you successfully subscribe to a data stream, Cortex will keep sending you data sample objects.

A subscription is linked to a session. All the subscriptions of a session are automatically cancelled when the session is closed. You can call unsubscribe to cancel a subscription.

The available data streams depend on the license of the user, and the model and settings of the headset. For an EPOC Flex, it also depends on the EEG sensor mapping.


## Data streams availability
Depending on the license and the model of the headset, some streams may be available or not, and may send data samples objects at a different rate.

| Stream | Supported headsets | Sample rate in hertz | 
| -------- | ------- | ------- |
| eeg | All, but requires a license | 128 or 256, depends on the headset and its settings. The license must contain the scope "eeg". You must activate the session before you subscribe. Please check the documentation of your headset. |
| mot  | All. | Disabled, 32, 64, 128, depends on the headset and its settings. 6.4 hertz for MN8. Please check the documentation of your headset. |
| dev | All | 2 |
| eq | All | 2 |
| pow | All | 8 |
| met | All. EPOC Flex requires a special EEG sensor mapping | 2 if the license contains the scope "pm" and you activate the session before you subscribe. 0.1 otherwise (1 sample every 10 seconds) |
| com | All. EPOC Flex requires a special EEG sensor mapping | 8 |
| fac | All but MN8. EPOC Flex requires a special EEG sensor mapping | 32 |
| sys | All | No fixed rate, see BCI for details. |


## Special EEG sensor mapping for EPOC Flex
By default, the streams "met", "com" and "fac" are not available for EPOC Flex. It is because Cortex doesn't implement the detections (performance metrics, mental command, facial expression) for EPOC Flex.

However, you can configure your EPOC Flex to simulate an EPOC X headset. In that case, Cortex runs the detections that are designed for EPOC X.

All you have to do is to use a sensor mapping that includes all the 14 EEG sensors of an EPOC X headset: AF3, F7, F3, FC5, T7, P7, O1, O2, P8, T8, FC6, F4, F8, AF4.
In addition, you should use P3 and P4 as references CMS and DRL. Please see the EPOC X [user manual](https://emotiv.gitbook.io/epoc-x-user-manual/introduction/introduction-to-epoc-x/coverage) for the complete EPOC X configuration.

Your mapping can also include sensors that are not present on an EPOC X. Cortex will not use these additional sensors to run the detections, but you can use them to collect more EEG data.

You must configure the EEG sensor mapping of your EPOC Flex when you call the method [controlDevice](https://emotiv.gitbook.io/cortex-api/headset/controldevice).