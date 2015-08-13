include($$[STARLAB])
include($$[SURFACEMESH])
include($$[CHOLMOD])


StarlabTemplate(plugin)

 QMAKE_CXXFLAGS += -openmp -arch:AVX -D "_CRT_SECURE_NO_WARNINGS"
 QMAKE_CXXFLAGS_RELEASE *= -O2

FORMS += \
    mess_widget.ui 
	
RESOURCES += scene_mess.qrc

HEADERS += \
    mess_widget.h \
	mess_mode.h \
	Geometry/Scene.h \
	Geometry/CModel.h \
	Geometry/SimplePointCloud.h \
	Geometry/UDGraph.h \
	Geometry/SceneRG.h \
	Geometry/QuickMeshDraw.h \
	Geometry/AABB.h \
	Geometry/OBB.h \
	Geometry/OBBEstimator.h \
	Geometry/OBBOBBIntersect.h \
	Geometry/BestFit.h \
	Geometry/ShapeLib.h \
	Geometry/TriTriIntersect.h \
	Geometry/SuppPlane.h\
	Geometry/SuppPlaneBuilder.h\
	Geometry/BoundingBox.h\
	Geometry/font.inl\
	Utilities/utility.h \
	Utilities/CustomDrawObjects.h \
	Math/MatrixType.h \
	Math/mathlib.h \
	Math/Eigen3x3.h 

SOURCES += \
    mess_widget.cpp \
	mess_mode.cpp \
	Geometry/Scene.cpp \
	Geometry/CModel.cpp \
	Geometry/SimplePointCloud.cpp \
	Geometry/UDGraph.cpp \
	Geometry/SceneRG.cpp \
	Geometry/AABB.cpp \
	Geometry/OBB.cpp \
	Geometry/OBBEstimator.cpp \
	Geometry/OBBOBBIntersect.cpp \
	Geometry/BestFit.cpp \
	Geometry/TriTriIntersect.cpp \
	Geometry/SuppPlane.cpp\
	Geometry/SuppPlaneBuilder.cpp\
	Geometry/BoundingBox.cpp\	
	Math/mathlib.cpp 



# CGAL and Boost
	CGAL_DIR = $$(CGAL_DIR)
	DEFINES += CGAL_CFG_NO_CPP0X_VARIADIC_TEMPLATES
	DEFINES += CGAL_CFG_NO_NEXTAFTER
	DEFINES += CGAL_CFG_NO_TR1_ARRAY
	DEFINES += CGAL_CFG_NO_TR1_TUPLE
	INCLUDEPATH *= $(CGAL_DIR)\include
	INCLUDEPATH *= $(CGAL_DIR)\build\include
	INCLUDEPATH *= $(CGAL_DIR)\auxiliary\gmp\include
	LIBS *= -L""$$CGAL_DIR"\build\lib"
	LIBS *= -lCGAL-vc120-mt-gd-4.5 -lCGAL-vc120-mt-4.5 -lCGAL_Core-vc120-mt-gd-4.5 -lCGAL_Core-vc120-mt-4.5
	LIBS *= -L""$$CGAL_DIR"/auxiliary/gmp/lib"
	LIBS *= -llibgmp-10
	INCLUDEPATH *= D:\boost_1_57_0
	LIBS *= -L"D:\boost_1_57_0\lib64-msvc-12.0"


