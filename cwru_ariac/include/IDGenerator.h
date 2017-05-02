//
// Created by tianshipei on 5/1/17.
//

#ifndef CWRU_ARIAC_IDGENERATOR_H
#define CWRU_ARIAC_IDGENERATOR_H

#include <atomic>

class IDGenerator {
public:
    static int genId() {
        static std::atomic<int> uid(0);  // id assigned from 0
        return ++uid;
    }

    static int genFakeId(int beginAt = 10000) {
        static std::atomic<int> uid(beginAt);
        return ++uid;
    }
};


#endif //CWRU_ARIAC_IDGENERATOR_H
