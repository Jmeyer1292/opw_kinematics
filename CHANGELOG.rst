^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opw_kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2021-04-23)
------------------
* Do not add compiler option -mno-avx if processor is uknown
* Contributors: Levi Armstrong

0.4.2 (2021-04-15)
------------------
* Do not add compiler option -mno-avx for arm
* Contributors: Levi Armstrong

0.4.1 (2021-04-09)
------------------
* Only enable initialize_code_coverage if code coverage is enabled
* Add cpack archive package
* Add package debian github action leveraging cpack
* Add CONTRIBUTING.md
* Contributors: Levi Armstrong

0.4.0 (2021-02-19)
------------------
* Vectorize using Eigen::Array and Eigen::Map
* Add code coverage CI build
* Clean CMakeLists.txt
* Switch to using std::array versus raw pointer to array
* Contributors: Levi Armstrong

0.3.1 (2021-01-06)
------------------
* Update to use initialize_code_coverage() macro
* Contributors: Levi Armstrong

0.3.0 (2021-01-06)
------------------
* Extract package name and version from package.xml
* Update CI build and add badges to readme
* Remove -std=c++11 compile option. This causes problems in downstream packages that add cxx_std_14 to compile features.
* Increase version to 0.2.0
* Add github actions
* Add code coverage to targets
* Add ros_industrial_cmake_boilerplate
* Contributors: Levi Armstrong, Matthew Powelson

0.1.0 (2020-02-06)
------------------
* Change build tool depend from catkin to cmake
* Change Affine3d to Isometry3d in readme
* Address compiler warnings
* Export targets with namespace to build directory
* Fix gtest target names
* Use new source_bat.bash command to set env variables (`#34 <https://github.com/Jmeyer1292/opw_kinematics/issues/34>`_)
  * Use source_bat to configure env variables
  * Update source_bat in travis.yml
  * Update source_bat in travis.yml
  * Remove comments and set build type to Release in travis.yml
* Add windows build to ci
* Add package specific gtest subdirectory
* Initialize Forward return value to identity
* Fix unit tests method for including gtest to work if exists
* Pure cmake (`#25 <https://github.com/Jmeyer1292/opw_kinematics/issues/25>`_)
  * Make opw_kinematics a pure cmake package
  * Set cmake version to 3.8.0
  * Add support for cmake versions on xenial
  * Replace cxx property with compiler option
  * Add namespace to exported targets
  * Add ExternalProject_add for GTest if not found
* force evaluation of Vector u in forward, fixes `#21 <https://github.com/Jmeyer1292/opw_kinematics/issues/21>`_
  using auto here allowed the compiler to create some sort of computation-object that
  was later handled incorrectly. Not sure if this is a bug in Eigen or g++. It also fixes
  a warning about uninitialized values being used and makes the tests succeed on our
  systems.
* switch to industrial_ci, add eigen dependency
* accept warnings as they stem from ikfast mostly
* Add travis config based on moveit_ci
* Fix printing of joint sign corrections. Print the sign correction as `-1` or `1` instead of the raw character.
* Replaced Affine with Isometry in Readme.md
* Changed license to Apache 2.0
* Replaced Affine with Isometry to better reflect what the solver is working toward
* Added image with positive rotations marked on it
* Added note about the default rotational axis
* Added noexcept to IK & FK calls. Casted all real number literals to the appropriate type. This prevents us from converting doubles to floats all the time and increases speed by quite a bit when you want to use floats.
* Swapped the short int to a signed char. We're just using it for -1 or 1 so it shouldn't matter.
* Made updates to the attribution for the idea behind the test: thanks jeroen demaeyer!
* Added a 'throughput' test for FK/IK for three different robots. No assertions but its nice to have something easy to re-run.
* Added a new set of tests based on JeroenDM's kuka tests that compute FK, solve the IK, and then make sure a new FK matches the original
* Renames abb2400_tests to abb2400_ikfast_tests to better capture the intent of the tests
* Moved the gtests inside a CATKIN_ENABLE_TESTING if-clause in the main cmake file
* Add joint sign corrections to parameters
* add ik test for using a single forward kinematics solution
* add simple fk test using a known solution for KukaKR6_R700_sixx
* Update KukaKR6_R700_sixx to match description kuka_experimental package
* add joint sign corrections to parameters and update io function
* Create LICENSE  GPLv3
* Moved some includes around
* Added a doc string pointing to the examples
* Added a few more example robot configurations.
* Documentation - noted that you can store the results in many different formats.
* Added diagram from paper; expanded examples
* Converted all the fixed double math over to templatized code.
* Added some utility headers for harmonization and validity checking.
* Added some documentation
* Added some basic unit tests to compare the OPW and IKFast solutions for the same abb 2400
* Contributors: CraigLin, G.A. vd. Hoorn, Jeroen, John Wason, Jonathan Meyer, Levi Armstrong, Matthew Powelson, Michael Ripperger, Simon Schmeisser, jeroendm
