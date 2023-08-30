
## Printing Thought Process
Dyllon and I decided to try and manufacture everything ourselves as much as possible. This meant doing a lot of research on how best to manufacture because water is an unforgiving environment and we don't need to destroy our equipment while trying to use it for the intended purpose. 

That being said, there are 2 primary methods of 3D printing. FDM and SLA printing. FDM is much more common - and it is the de facto method as well. This involves extruding heated PLA, ABS, or other solid materials onto a heated bed. This is a great method for rapid prototyping as it is easy to postprocess and generally tends to print quickly SLA printing is a little more involved. For starters, SLA involves photopolymer resin and an LCD screen to cure. This allows very tight tolerances on prints, but the print process is much longer. Most notably, the post-processing task is very involved. Once the print is done, the operator has to first and foremost wear safety glasses, gloves, and a breathing mask because of the toxic nature of photopolymer resin. Then the print must be removed from the printer and placed in an isopropyl alcohol bath which can take anywhere from 30 seconds to 10 minutes. I opted to keep the wash time low, but that is mostly because there is a fine line to walk between washing it so much that the process time is even longer because the excess IPA has to dry, and not rinsing the excess resin off before trimming supports and curing. 

Although it sounds like FDM printing is the way to go in this case, take note that layer lines are a big deal in this application. On FDM printers you can almost see each layer line (obviously depending on what size nozzle is used). However, in SLA printing the layer lines are significantly smaller. As in, anywhere from 25 to 100 microns. This is very important for keeping water out. A quick Reddit scour shows that when folks print with FDM for water retention purposes, one of the first failure points is the layer line. Having such a small layer line on an SLA print makes this a much smaller likelihood. Additionally, SLA allows for clear and translucent prints, which is important where the entire robot is custom designed and built. There aren't any prebuilt observation domes. That had to be built by us as well.

This is all to say we opted for SLA printing even though it is a more involved process simply because the end product is likely to be of higher quality, and the robust nature of cured photopolymer resin is desirable over FDM. 
## Software
Just like in FDM printing, a slicer and modeler is required. For simplicity, Dyllon and I used [tinkerCAD](https://www.tinkercad.com/login) to model each individual component. Although there is a whole slew of more powerful modeling softwares out there like Blender, tinkerCAD has the smallest learning curve, and for our purposes works just fine. 

As slicers are concernced, I opted to use what Elegoo recommends, which is [ChituBOX](https://www.chitubox.com/en/index). Some folks use others, but as an entrance into SLA printing this software works fine and had a shallow learning curve.
## Materials and Equipment Used
Since Dyllon and I are really wanting this to be something people can build or modify at home, I think it is important to list basically everything we used. 

Equipment:
- [Elegoo Saturn 2 8K](https://www.elegoo.com/products/elegoo-saturn-2-8k-10-inches-mono-lcd-3d-printer)
- [Elegoo Wash/Cure Station](https://www.elegoo.com/products/elegoo-mercury-x-bundle-washing-and-curing-machine)
- Rubber Gloves (disposable latex gloves are also fine)
- Safety Glasses (It is very easy for IPA or rogue splintering pieces to find their way into your unprotected eyes - ask me how I know)
- Sanding Paper 
     - This is an essential post processing step for clear resins. In my experience thus far I recommend grits from 120-10,000. This allows us to really work out any imperfections in the print and especially make the dome clear like glass.

Materials:
- [Elegoo Translucent Resin](https://www.amazon.com/dp/B07Z994PW2?ref=nb_sb_ss_w_as-reorder-t1_k0_1_24&amp=&crid=R9T3GDTHDA74&sprefix=elegoo%2Btranslucent%2Bresin&th=1) Elegoo has been making resin for a long time. Their formula is pretty tried and true, and even if you don't have your slicer settings completely dialed in it still tends to yield pretty fair results.
     - I recommend about 5-7 kilograms of this resin because it comprises almost the entire hull (and it is good to account for any mistakes, trust me I made many whilst printing)
     - It is not entirely necessary that translucent resin be used for the hull or connectors. Those can be any color you like! Dyllon and I just wanted to be able to see all of the internals as well.
- [Siraya Tech Resin](https://www.amazon.com/Siraya-Tech-Ultra-Clear-Non-Yellow-Transparency/dp/B09LQW7RHX/ref=sr_1_4_pp?crid=CRT72PHRS2ID&keywords=clear%2Bresin&qid=1693422767&s=industrial&sprefix=clear%2Bresin%2Cindustrial%2C138&sr=1-4&th=1) 
     - This resin yielded a very clear observation dome. There may be better alternatives out there, but this more than does the job.
     - I recommend 1 kilogram of the stuff unless you want to make more components clear. Do note that this resin is a bit harder to use than Elegoo, so if you are not entirely confident in your SLA printing skills consider opting for an alternative. 
## Post-Processing
Please note that while I do have *some* eperience with SLA 3D printing, I am by no means an expert. Some of my processes may look extremely barbaric to those who are more well versed, but the outcome was of at least acceptable quality. 

I found that anywhere from 30 seconds to 2 minutes was sufficient to wash the excess resin off.

Next comes trimming supports. There isn't a particular time frame with this, the only advice I can offer is to take things extrememly slowly. Resin is generally stronger after being cured, but where it hasn't been fully cured yet, it is prone to chipping off. This is especially true near the connection point between supports and the actual print.

Finally, curing is necessary. For the curing machine that I listed, I like to insert the model for 30 seconds each time. You can tell when it is cured because it will not be tacky to the touch. If it is, hit it with another 30 seconds. The curing machine could be set to more time, but it is very important not to overexpose the translucent or clear resins, as they will begin to turn yellow. 

Once these steps are complete and all parts are printed, I recommend going to the local hardware store or Home Depot and picking up some clear resin/polyurethane. Make sure it is flat/low gloss just to make sure there isn't any undesired shine bouncing back into the camera. The poly/clear resin is a good final coat to fill in any minor imperfections in the print and further reduce the likelihood of a leak through the hull. Do not be bashful with the application of the exterior coats, the more the merrier. If it accidentally dries globbed, just sand off the mistake and reapply. 