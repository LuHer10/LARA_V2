QT += quick network

CONFIG += c++11

SOURCES += \
    main.cpp \
    UDPHandler.cpp

HEADERS += \
    UDPHandler.h

RESOURCES += qml.qrc

DISTFILES += \
    ../JoystickApp.pro/AndroidManifest.xml \
    ../JoystickApp.pro/build.gradle \
    ../JoystickApp.pro/gradle.properties \
    ../JoystickApp.pro/gradle/wrapper/gradle-wrapper.jar \
    ../JoystickApp.pro/gradle/wrapper/gradle-wrapper.properties \
    ../JoystickApp.pro/gradlew \
    ../JoystickApp.pro/gradlew.bat \
    ../JoystickApp.pro/res/values/libs.xml \
    ../JoystickApp.pro/res/xml/qtprovider_paths.xml \
    android/AndroidManifest.xml \
    android/build.gradle \
    android/res/values/libs.xml \
    android/res/xml/qtprovider_paths.xml

contains(ANDROID_TARGET_ARCH,arm64-v8a) {
    ANDROID_PACKAGE_SOURCE_DIR = \
        $$PWD/../JoystickApp.pro
}
