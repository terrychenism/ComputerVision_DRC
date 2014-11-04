#include "permutohedral.h"
#include "filter.h"

Filter::Filter( const float * source_features, int N_source, const float * target_features, int N_target, int feature_dim ):n1_(N_source),o1_(0),n2_(N_target), o2_(N_source){
	permutohedral_ = new Permutohedral();
	float * features = new float[ (N_source+N_target)*feature_dim ];
	memcpy( features, source_features, N_source*feature_dim*sizeof(float) );
	memcpy( features+N_source*feature_dim, target_features, N_target*feature_dim*sizeof(float) );
	permutohedral_->init( features, feature_dim, N_source+N_target );
	delete[] features;
}
Filter::Filter( const float * features, int N, int feature_dim ):n1_(N),o1_(0),n2_(N), o2_(0){
	permutohedral_ = new Permutohedral();
	permutohedral_->init( features, feature_dim, N );
}
Filter::~Filter(){
	delete permutohedral_;
}
void Filter::filter( const float * source, float * target, int value_size ){
	permutohedral_->compute( target, source, value_size, o1_, o2_, n1_, n2_ );
}
void Filter::reverseFilter( const float * source, float * target, int value_size ){
	permutohedral_->compute( target, source, value_size, o2_, o1_, n2_, n1_ );
}

