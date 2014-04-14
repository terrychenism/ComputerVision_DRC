    #include <iostream>
    #include <Eigen/Geometry>
    #include <Eigen/Dense>

	using namespace std;
    // int main(int argc, char **argv)
    // {
    //     //Below is setup stuff
    //     Eigen::Vector3f v1(Eigen::Vector3f::Zero());
    //     Eigen::Matrix3f m1(Eigen::Matrix3f::Identity());
    //     m1(0,0) = 2.0*2.0;
    //     m1(2,2) = 0.5*0.5;

    //     Eigen::AngleAxisf rot(0.25*M_PI, Eigen::Vector3f::UnitZ());


    //     Eigen::Matrix3f m1_rot = rot.toRotationMatrix().transpose() * m1 * rot.toRotationMatrix();
    //     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(m1_rot);

    //     const Eigen::Vector3f& eigValues (eig.eigenvalues());
    //     const Eigen::Matrix3f& eigVectors (eig.eigenvectors());

    //     //The important part: Construct a quaternion from the matrix of eigenvectors (which is a rotation matrix)
    //     Eigen::Quaternionf quaternion (eigVectors);

    //     std::cout << "\neigVec:\n" << eigVectors << "\n";

    //     std::cout << "\nw:" << quaternion.w() << " x:" << quaternion.x() << " y:" << quaternion.y() << " z:" << quaternion.z() << "\n";   
    // }

int main() {


  //Eigen::Matrixm q(1,4);
  // Eigen::Matrix3d p;
  // p << 1, 2, 3,
  //      4, 5, 6,
  //      7, 8, 9;
  //      cout<<p<<endl;
   Eigen:: Vector4d q(-0.966, 0.259,0.0, 0.0);
// q<<0.966, 0.259,0.0, 0.0;
  	//cout<<q.transpose()<<endl;

  	Eigen::Matrix<double,1,4> io= q.transpose();
  	cout<<io<<endl;

  	//cout<<io.quaternion.toRotationMatrix()<<endl;

Eigen::Quaternionf e_q(0.966-M_PI,0.259,0,0);
//quaternion<<0.966, 0.259,0.0, 0.0;

Eigen::Matrix3f rot=e_q.toRotationMatrix();
cout<<rot<<endl;
//cv::Mat cv_rot=cv::Mat_<float>(4,4)<<(rot(0,0), rot(0,1)

// float qw = quaternion.R_component_1();
// float qx = quaternion.R_component_2();
// float qy = quaternion.R_component_3();
// float qz = quaternion.R_component_4();
  	//cout<<io.toRotationMatrix()<<endl;
  // Eigen::Matrix n <Scalar,4,1> = q.transpose();;
  // std::cout<<n<<std::endl;
  // Matrix/matrix multiplication
  // std::cout << "p*p:\n" << p*p << std::endl;

  // // Matrix/vector multiplication
  // std::cout << "p*r:\n" << p*r << std::endl;
  // std::cout << "r^T*p:\n" << r.transpose()*p << std::endl;
  
  // // Vector/vector multiplication (inner product)
  // std::cout << "r^T*s:\n" << r.transpose()*s << std::endl;
}
