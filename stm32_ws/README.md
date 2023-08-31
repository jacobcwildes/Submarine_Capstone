
# STM Board Decisionmaking

Since the project is entirely self contained, power is a concern. That being said, we knew that we needed as much power efficiency as possible. According to [Digi-Key's](https://www.digikey.com/en/maker/blogs/2020/understanding-stm32-naming-conventions) website, the flag after "STM" is the type. We opted for two low power boards. 

As far as the controller STM board is concerned, there are not many things that need to be driven or read from. There is only one button for a screenshot, a switch to control up/down, and 4 ADC channels. That being the case,
we opted for the STM32-L452RE-P. This was primarily because it was already on hand from a previous class, and met the criteria of being low power. 

Had we not had that board on hand, a good alternative would have been the [Nucleo-L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html#st_all-features_sec-nav-tab) as it still has the same functionality, is still low power, but has even fewer wasted pins than on the currently selected board. 
