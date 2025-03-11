#include "localization/ParticleFilter.h"
#include "localization/Util.h"

#include "tf/tf.h"
#include <iostream>

using namespace std;

ParticleFilter::ParticleFilter(int numberOfParticles) {
	this->numberOfParticles = numberOfParticles;

	// initialize particles
	for (int i = 0; i < numberOfParticles; i++) {
		this->particleSet.push_back(new Particle());
	}

	// this variable holds the estimated robot pose
	this->bestHypothesis = new Particle();

	// at each correction step of the filter only the laserSkip-th beam of a scan should be integrated
	this->laserSkip = 5;

	// distance map used for computing the likelihood field
	this->distMap = NULL;
}

ParticleFilter::~ParticleFilter() {
	// delete particles
	for (int i = 0; i < numberOfParticles; i++) {
		Particle* p = this->particleSet[i];
		delete p;
	}

	this->particleSet.clear();

	if (this->likelihoodField)
		delete[] this->likelihoodField;

	delete this->bestHypothesis;

	if (this->distMap)
		delete[] this->distMap;
}

int ParticleFilter::getNumberOfParticles() {
	return this->numberOfParticles;
}

std::vector<Particle*>* ParticleFilter::getParticleSet() {
	return &(this->particleSet);
}

void ParticleFilter::initParticlesUniform() {
    //get map properties
    int mapWidth, mapHeight;
    double mapResolution;
    this->getLikelihoodField(mapWidth, mapHeight,mapResolution);

	// TODO: here comes your code
    //initialize variables to temporaly store 
    //the random x, y, theta
    double randx;
    double randy;
    double randth;

	for(int i=0; i<this->numberOfParticles;i++)
	{
		randx=Util::uniformRandom(0, mapWidth*mapResolution); 
		randy=Util::uniformRandom(0, mapHeight*mapResolution);
		randth=Util::uniformRandom(0, 2.0*M_PI);
		this->particleSet[i] = new Particle(randx, randy, randth, 1.0/(double)(this->numberOfParticles));
	}

}

void ParticleFilter::initParticlesGaussian(double mean_x, double mean_y,
		double mean_theta, double std_xx, double std_yy, double std_tt) {
    
    //initialize variables to temporaly store 
    //the gaussian sample  x, y, theta
    double gauss_sample_x;
    double gauss_sample_y;
    double gauss_sample_th;
	
    //iterate through all particles and initialize with gaussian sample
	for(int i=0; i<this->numberOfParticles;i++)
	{
		gauss_sample_x=Util::gaussianRandom(mean_x, std_xx); 
		gauss_sample_y=Util::gaussianRandom(mean_y, std_yy);
		gauss_sample_th=Util::gaussianRandom(mean_theta, std_tt);
		this->particleSet[i]=new Particle(gauss_sample_x,gauss_sample_y, gauss_sample_th, 1.0/(double)(this->numberOfParticles));
	}
}

/**
 *  Initializes the likelihood field as our sensor model.
 */
void ParticleFilter::setMeasurementModelLikelihoodField(
    const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit) {
    ROS_INFO("Creating likelihood field for laser range finder...");

    // Dimensions and resolution of the likelihood field
    this->likelihoodFieldWidth = map.info.width;
    this->likelihoodFieldHeight = map.info.height;
    this->likelihoodFieldResolution = map.info.resolution;

    // Allocate memory for the likelihood field
    this->likelihoodField = new double[likelihoodFieldWidth * likelihoodFieldHeight];

    // Compute the distance map
    calculateDistanceMap(map);

	// Here you have to create your likelihood field
	// HINT0: sigmaHit is given in meters. You have to take into account the resolution of the likelihood field to apply it.
	// HINT1: You will need the distance map computed 3 lines above
	// HINT2: You can visualize it in the map_view when clicking on "show likelihood field" and "publish all".
	// HINT3: Storing probabilities in each cell between 0.0 and 1.0 might lead to round-off errors, therefore it is
	// good practice to convert the probabilities into log-space, i.e. storing log(p(x,y)) in each cell. As a further
	// advantage you can simply add the log-values in your sensor model, when you weigh each particle according the
	// scan, instead of multiplying the probabilities, because: log(a*b) = log(a)+log(b).

	// TODO: here comes your code

    //probability mixture, from slides
    double zHit = 1.0 - zRand;
    //scale sigmaHit according to Map resolution
    sigmaHit = sigmaHit/this->likelihoodFieldResolution ;

    //fill the likelihood field
    for (int y = 0; y < likelihoodFieldHeight; ++y) {
        for (int x = 0; x < likelihoodFieldWidth; ++x) {
            int index = computeMapIndex(this->likelihoodFieldWidth,this->likelihoodFieldHeight,x,y);
            double distInMeters = this->distMap[index]/this->likelihoodFieldResolution;  //distance to obstacles in meters

            // pHit is a gaussian probability
            double pHit = Util::gaussian(distInMeters,sigmaHit,0);

            //combined probability, formula from slides, pRand = 1, zmax=pmax=0
            double combinedProbability = zHit * pHit + zRand; 
            this->likelihoodField[index] = log(combinedProbability);

        }
    }

    ROS_INFO("...DONE creating likelihood field!");
}

void ParticleFilter::calculateDistanceMap(const nav_msgs::OccupancyGrid& map) {
	// calculate distance map = distance to nearest occupied cell
	distMap = new double[likelihoodFieldWidth * likelihoodFieldHeight];
	int occupiedCellProbability = 90;
	// initialize with max distances
	for (int x = 0; x < likelihoodFieldWidth; x++) {
		for (int y = 0; y < likelihoodFieldHeight; y++) {
			distMap[x + y * likelihoodFieldWidth] = 32000.0;
		}
	}
	// set occupied cells next to unoccupied space to zero
	for (int x = 0; x < map.info.width; x++) {
		for (int y = 0; y < map.info.height; y++) {
			if (map.data[x + y * map.info.width] >= occupiedCellProbability) {
				bool border = false;
				for (int i = -1; i <= 1; i++) {
					for (int j = -1; j <= 1; j++) {
						if (!border && x + i >= 0 && y + j >= 0 && x + i
								< likelihoodFieldWidth && y + j
								< likelihoodFieldHeight && (i != 0 || j != 0)) {
							if (map.data[x + i + (y + j) * likelihoodFieldWidth]
									< occupiedCellProbability && map.data[x + i
									+ (y + j) * likelihoodFieldWidth] >= 0)
								border = true;
						}
						if (border)
							distMap[x + i + (y + j) * likelihoodFieldWidth]
									= 0.0;
					}
				}
			}
		}
	}
	// first pass -> SOUTHEAST
	for (int x = 0; x < likelihoodFieldWidth; x++)
		for (int y = 0; y < likelihoodFieldHeight; y++)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}

	// second pass -> NORTHWEST
	for (int x = likelihoodFieldWidth - 1; x >= 0; x--)
		for (int y = likelihoodFieldHeight - 1; y >= 0; y--)
			for (int i = -1; i <= 1; i++)
				for (int j = -1; j <= 1; j++)
					if (x + i >= 0 && y + j >= 0 && x + i
							< likelihoodFieldWidth && y + j
							< likelihoodFieldHeight && (i != 0 || j != 0)) {
						double v = distMap[x + i + (y + j)
								* likelihoodFieldWidth] + ((i * j != 0) ? 1.414
								: 1);
						if (v < distMap[x + y * likelihoodFieldWidth]) {
							distMap[x + y * likelihoodFieldWidth] = v;
						}
					}
}

double* ParticleFilter::getLikelihoodField(int& width, int& height,
		double& resolution) {
	width = this->likelihoodFieldWidth;
	height = this->likelihoodFieldHeight;
	resolution = this->likelihoodFieldResolution;

	return this->likelihoodField;
}

/**
 *  A generic measurement integration method that invokes some specific observation model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::measurementModel(
		const sensor_msgs::LaserScanConstPtr& laserScan) {
	likelihoodFieldRangeFinderModel(laserScan);
}

/**
 *  Method that implements the endpoint model for range finders.
 *  It uses a precomputed likelihood field to weigh the particles according to the scan and the map.
 */
void ParticleFilter::likelihoodFieldRangeFinderModel(
		const sensor_msgs::LaserScanConstPtr & laserScan) {

	// TODO: here comes your code

    vector<Particle*>* particleSet = this->getParticleSet();

    for (Particle* particle : *particleSet) {
        double logWeight = 0.0;

        for (size_t i = 0; i < laserScan->ranges.size(); i += this->laserSkip) {
            double range = laserScan->ranges[i];
            
            //skip invalid ranges including NANs and measurements outside of range
            if(isnan(range)) continue;
            if(range < laserScan->range_min || range > laserScan->range_max) continue; 
            
            //compute beams endpoint in similar way as in mapping task 
            double scanAngle = particle->theta + laserScan->angle_min + i * laserScan->angle_increment;
            
            double endX = particle->x + range * cos(scanAngle);
            double endY = particle->y + range * sin(scanAngle);

            //get likelihood field index
            int endXfield = (int)(endX / this->likelihoodFieldResolution);
            int endYfield = (int)(endY / this->likelihoodFieldResolution);

            //check if the endpoint is within map bounds
            if (endXfield >= 0 && endXfield < this->likelihoodFieldWidth && endYfield >= 0 && endYfield < this->likelihoodFieldHeight) {
                int index = endYfield * this->likelihoodFieldWidth + endXfield;
                logWeight += this->likelihoodField[index]; //add log-probability from likelihood field
            } else {
                logWeight -= 1.0; //penalizes beams hitting outside the map
                continue;
            }
        }
	
        particle->weight = logWeight;


    }

}

void ParticleFilter::setMotionModelOdometry(double alpha1, double alpha2,
		double alpha3, double alpha4) {
	this->odomAlpha1 = alpha1;
	this->odomAlpha2 = alpha2;
	this->odomAlpha3 = alpha3;
	this->odomAlpha4 = alpha4;

}

/**
 *  A generic motion integration method that invokes some specific motion model.
 *  Maybe in the future, we add some other model here.
 */
void ParticleFilter::sampleMotionModel(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	sampleMotionModelOdometry(oldX, oldY, oldTheta, newX, newY, newTheta);
}

/**
 *  Method that implements the odometry-based motion model.
 */
void ParticleFilter::sampleMotionModelOdometry(double oldX, double oldY,
		double oldTheta, double newX, double newY, double newTheta) {
	// TODO: here comes your code
           //odometry calculation from slides
    double deltaRot1 = Util::normalizeTheta(atan2(newY - oldY, newX - oldX) - oldTheta);
    double deltaTrans = sqrt(pow(newX - oldX, 2) + pow(newY - oldY, 2));
    double deltaRot2 = Util::normalizeTheta(newTheta - oldTheta - deltaRot1);

    vector<Particle*>* particles = this->getParticleSet();

    //update each particle
    for (Particle* particle : *particles) {
        //noise model from slides
        double noisyRot1 = deltaRot1 + Util::gaussianRandom(0.0, this->odomAlpha1 * fabs(deltaRot1) + this->odomAlpha2 * fabs(deltaTrans));
        double noisyTrans = deltaTrans + Util::gaussianRandom(0.0, this->odomAlpha3 * fabs(deltaTrans) +this->odomAlpha4 * fabs(deltaRot1 + deltaRot2));
        double noisyRot2 = deltaRot2 + Util::gaussianRandom(0.0, this->odomAlpha1 * fabs(deltaRot2) + this->odomAlpha2 * fabs(deltaTrans));
        //update particle pose according to motionmodel
        particle->x += noisyTrans * cos(particle->theta + noisyRot1);
        particle->y += noisyTrans * sin(particle->theta + noisyRot1);
        particle->theta = Util::normalizeTheta(particle->theta + noisyRot1 + noisyRot2);
    }
}

/**
 *  The stochastic importance resampling.
 */
void ParticleFilter::resample() {
    vector<Particle*>* particleSet = this->getParticleSet();

    
    //converting particle weight to positive weights which exp and normalizing for 
    //better application of stochastic universal resampling algorithm
    
    //calculate sum of exp weights
    double sumWeights = 0.0;
    for (Particle* particle : *particleSet) {
        sumWeights += exp(particle->weight);
    }
    
    Particle* MaxWeightParticle;
    //variable sacing the max weight
    double maxResampledWeight = 0;
    //normalize the particle weights and find best hypothesis particle with max weight
    for (Particle* particle : *particleSet) {
        
        particle->weight = exp(particle->weight)/sumWeights;
        
        //find best hypothesis particle with max weight
        if (particle->weight > maxResampledWeight)
        {
            MaxWeightParticle=particle;
            maxResampledWeight=particle->weight;
        }
    }
    
    //update previous best hypothesis
    this->bestHypothesis = MaxWeightParticle;

    
    //In the following the universal stochastic resampling algorithm is implemented
    
    //generate CDF in form of a vector 
    vector<double> cumulativeWeights(this->getNumberOfParticles(), 0.0);
    cumulativeWeights[0] = (*particleSet)[0]->weight;
    for (int i = 1; i < this->getNumberOfParticles(); ++i) {
        cumulativeWeights[i] = cumulativeWeights[i - 1] + (*particleSet)[i]->weight;
    }

    //calculate the threshold
    double resamplingInterval = 1.0 / this->getNumberOfParticles();
    //initialize sampled starting position
    double start = Util::uniformRandom(0.0, resamplingInterval);
    vector<Particle*> resampledParticles;
    //index for iterating cumulativeWeights
    int index = 0;
        
    double threshold;

    // Resample particles
    for (int i = 0; i < this->getNumberOfParticles(); ++i) {
        // Increment Threshold
        threshold = start + i * resamplingInterval;
        
        //skip until next threshold reached
        while (index < this->getNumberOfParticles() - 1 && threshold > cumulativeWeights[index]) ++index;
        
    	//add new sampled particle
        resampledParticles.push_back(new Particle(this->particleSet[index]->x, this->particleSet[index]->y, this->particleSet[index]->theta, 1.0/(double)this->numberOfParticles));
    }

    //update particle set with resampled particles
    this->particleSet=resampledParticles;
}

Particle* ParticleFilter::getBestHypothesis() {
	return this->bestHypothesis;
}

// added for convenience
int ParticleFilter::computeMapIndex(int width, int height, int x,
		int y) {
	return x + y * width;
}

