# stream_sin_wave.py

## Channel name explanation

- A channel name follows the following pattern: `channel_X_Y_Z`, where
    - `X`: is the number of samples present in each sinusoidal cycle
        - The larger this number is, the larger is the sample rate and thus we have a more faithful and smooth representation of the real curve (Nyquist–Shannon sampling theorem)

    - `Y`: represents the time step (in milliseconds) between 2 subsequent batches of data.
        - Data is sent in json format
        - Each "batch" streamed by the websocket contains one or more samples
        - Each "batch" streamed by the websocket is contained within square brackets and each sample within it is contained within curly brackets
        - If `Y` is set to 20, each new batch of data will be within 20 milli seconds from the previous one
        - The amount of samples within that batch depends on the values of `X` and `Z`
        - The larger this number is, more data will be received at once and the time without receiving new data will also be larger
            - Example: if you specify `Y=10000` you'll only receive 1 batch of data each 10 seconds, and the socket will be silent for around 10 seconds also

    - `Z`: is the frequency of the sinusoidal wave itself