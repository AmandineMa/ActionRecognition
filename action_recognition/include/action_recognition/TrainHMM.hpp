#ifndef TRAINHMM_HPP
#define TRAINHMM_HPP

/**
 * \file TrainHMM.hpp
 * \brief Namespace containing function to train an HMM
 * \author Amandine M.
 */

#include <vector>

#include "action_recognition/common.hpp"
#include "action_recognition/FeatureMatrix.hpp"
#include "action_recognition/Setup.hpp"

/** 
 * \brief Namespace gathering functions to initialize and train HMM
 */

namespace TrainHMM{

/** 
 *\brief Train 1 HMM *label* from the corresponding set of data
 *
 * The data files previously listed in a .scp file are retrieved from the label.
 * It is possible to perform 2 kind of initialization and 1 kind of parameter-estimation
 * \param label Name of the HMM to be trained, corresponding to an action name
 * \param print_output Print output or not on the terminal
 * \param isolated_unit_init Initialization with HInit or not
 * \param isolated_unit_flat_init Initialization with HCompV (with the set of data corresponding to the action and not the whole set of data) or not
 * \param isolated_unit_training Parameters re-estimation with HRest
 * \param emission_type #EmissionType
 * \param topology_type #TopologyType
 * \param dim Size of a feature vector
 * \param median_samp_nb Median value of the number of samples of data files
 * \param num_states_def State number calculated or defined by user 
 * \param iterations_nb Maximum of iterations to be performed by the training tools
 * \param setup #Setup
 * \param mixtures_nb Number of mixtures per state, by default 1
 **/
void train_HMM(std::string label, bool print_output, bool isolated_unit_init, bool isolated_unit_flat_init, bool isolated_unit_training, EmissionType emission_type, TopologyType topology_type, int dim, int median_samp_nb, StatesNumDef num_states_def, int iterations_nb, Setup setup, int mixtures_nb=1);


/**
 * \brief Train all the HMM with HERest
 *
 * Embedded training, parameter re-estimation for each HMM is based on the whole set of data 
 * and not only the corresponding action.
 * The data files previously listed in a .scp file are retrieved from the data_list_path from setup
 * \param print_output Print output or not on the terminal
 * \param setup #Setup
 **/
void train_HMMs(bool print_output, Setup setup);

/**
 * \brief Initialization of 1 HMM but based on the whole dataset with HCompv
 *
 * Embedded flat initialization, parameter initialization for each HMM is based on the whole set of data 
 * and not only the corresponding action.
 *
 * Must not be used with isolated_unit_init and isolated_unit_flat_init
 *
 * The data files previously listed in a .scp file are retrieved from the data_list_path from setup
 * \param print_output Print output or not on the terminal
 * \param setup #Setup
 * \param Name of the HMM to be trained
 * \param num_states_def State number calculated or defined by user 
 * \param dim Size of a feature vector
 * \param median_samp_nb Median value of the number of samples of data files
 * \param emission_type #EmissionType
 * \param topology_type #TopologyType
 * \param mixtures_nb Number of mixtures per state, by default 1
 **/
void embedded_unit_flat_init(bool print_output, Setup setup, std::string hmm_name, StatesNumDef num_states_def, int dim, int median_samp_nb, EmissionType emission_type, TopologyType topology_type, int mixtures_nb=1);

};
#endif
