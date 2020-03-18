# AudioLevelBeta
This is an improved beta version of the AudioLevel rainmeter plugin.

## Added features
Here's a list of improved/addded features of AudioLevelBeta compared to the original AudioLevel version: 
(Optimizations and fixes are not listed)
#### Silence detection
The plugin will have almost no CPU/GPU usage if it detects silence.
#### Dynamic Volume Adjustment
Same music visualization result for low- and high-volume music. No need anymore to adjust the sensitivity all the time!
To enable, put 
`DynamicVolume=1`
#### Update Rate Limiter
You can now limit the update rate (Recommended) as rainmeter can crash on the default unlimited framerate if multiple skins are loaded.
- `UpdatesPerSecond=60`: This will for example update the measures and meters 60 times per second. You can set this to any number under 240. (recommended)
- `UpdatesPerSecond=0`: This will disable the updating.
- `UpdatesPerSecond=-1`: Unlimited: The plugin updates as fast as possible. (default) (the highest CPU/GPU usage)
- `UpdatesPerSecond=-2`: This will disable the updating and the capture thread (legacy AudioLevel behavior). This is the fastest setting of all but the visualization will not look good most of the time.

Put the update bang for your measures and meters in `OnUpdateAction`, for example `OnUpdateAction=[!UpdateMeasureGroup Audio][!UpdateMeterGroup Bars]`
#### Wave and WaveBand Types
You can now set the AudioLevel parent measure type to `Wave` or `WaveBand`.
The difference between `Wave` and `WaveBand` is that Wave outputs the raw wave without scaling or smoothing.
I almost always recommend WaveBand.
##### Wave
Use the `WaveSize` option to set the WaveSize (similar to FFTSize), recommended values are: 512 to 8192, but you can go higher no problem.
Use WaveIdx on the child measures to assign them a number between 0 and WaveSize. (Just like FFTIdx)
##### WaveBand
Use the `WaveSize` option to set the WaveSize (similar to FFTSize), recommended values are: 512 to 8192, but you can go higher no problem.
Use the `Bands` option to specify on how much bands the wave should be scaled (for example 50), independent from `WaveSize`.
Jou can now use the `Smoothing` option.
Use BandIdx on the child measures to assign them a number from 0 to `Bands`.
#### Smoothing options
Measures of type `Band` or `WaveBand` can utilize this smoothing feature.
Use the `Smoothing` option to specify the amount of negibour values to build the average. For example 3 or 5.

## Contributers
- [SnGmng](https://github.com/SnGmng)
- [alatsombath](https://github.com/alatsombath)
- [dgrace](https://docs.rainmeter.net/manual/plugins/audiolevel/)
