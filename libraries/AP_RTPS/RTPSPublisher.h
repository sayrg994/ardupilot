/*
 * Copyright (C) 2016 Intel Corporation. All rights reserved.
 *
 * Based on a file auto-generated by the fastcdrgen tool with the following
 * Copyright and license:
 * Copyright (C) 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <string>

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>

#include "RTPSPubSubType.h"

template <class State>
class RTPSIsolatedPublisher {
public:
    RTPSIsolatedPublisher() {}
    ~RTPSIsolatedPublisher();

    bool init(const std::string& publisher, const std::string& topic);
    int publish_state(State& state);

private:
    class PubListener : public eprosima::fastrtps::PublisherListener {
    public:
        void onPublicationMatched(eprosima::fastrtps::Publisher *,
            eprosima::fastrtps::rtps::MatchingInfo&);

        int n_matched{0};
    } listener;

    eprosima::fastrtps::Participant *participant{nullptr};
    eprosima::fastrtps::Publisher *publisher{nullptr};

    RTPSPubSubType<State> *type{nullptr};
};
