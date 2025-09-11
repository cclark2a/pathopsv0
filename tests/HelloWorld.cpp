/*
* Copyright 2017 Google Inc.
*
* Use of this source code is governed by a BSD-style license that can be
* found in the LICENSE file.
*/
// (c) 2023, Cary Clark cclark2@gmail.com

#include "HelloWorld.h"

extern void OpTest();

using namespace sk_app;

Application* Application::Create(int argc, char** argv, void* platformData) {
    return new HelloWorld(argc, argv, platformData);
}

HelloWorld::HelloWorld(int argc, char** argv, void* platformData) {
    OpTest();
    exit(0);
}

void HelloWorld::onIdle() {
}
