# MCMC_Data_Association
This is clearly a work in progress. This is my attempted implementation of the Online MCMCDA algorithm described in the pdf.

Room for improvement: When calculating the posterior it'd be faster to take
the log and use summations instead of multiplication.

To do:  Likelihood function (in progress)
	propasal moves
	Markov Chain convergenace/ mixing time
	copy constructor for graph class (So we can store the MAP estimation)
/***********************************10/17/17******************************************/
Need to implement propsal moves. We need a modified version of it though, our Window is mainly going to be used as a memory 
and going to be immutable. Since we need to ship reliaable/consistent data back to the CASAS we restrict our proposal window
to 15 frames. The Kinect has 30 frames per second so this allows close to two messages per second. Might cut it down to 10 frames 
and lose to 3 frames.



Also the mixing time is going to be a constant for now. 
/***********************************************************************************/