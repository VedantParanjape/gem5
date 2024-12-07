/**
 * TODO
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/cache/replacement_policies/shepherd_rp.hh"

#include <cassert>
#include <memory>

#include "params/SHEPHERDRP.hh"
#include "sim/cur_tick.hh"

#include <iostream>

namespace gem5
{

namespace replacement_policy
{

SHEPHERD::SHEPHERD(const Params &p)
  : Base(p)
{
}

std::vector<std::vector<int>> SHEPHERD::SHEPHERDReplData::next_value_counter;

void
SHEPHERD::invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
{
    // Reset last touch timestamp
    if (!std::static_pointer_cast<SHEPHERDReplData>(replacement_data)->shepherd_cache_flag)
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->lastTouchTick = Tick(0);
    else
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->tickInserted = 0;
}

void
SHEPHERD::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // check if the CM[sc_index] == -1
    // if == -1, then set CM of all sc indexes to nvc[set] value
    // then increment nvc of all sc indexes (here all means the sc's which are populated)
    // this populated or not can be found by checking for nvc value, if it is -1 or 0
    // if -1 then it is unpopulated and we don't mess around
    // if 0 we do all this
    auto sc_repl_data = std::static_pointer_cast<SHEPHERDReplData>(replacement_data);
    for (int i = 0; i < sc_repl_data->count_value_matrix.size(); i++) {
        if (sc_repl_data->next_value_counter[sc_repl_data->set_index][i] != -1 && sc_repl_data->count_value_matrix[i] == -1) {
            sc_repl_data->count_value_matrix[i] = sc_repl_data->next_value_counter[sc_repl_data->set_index][i]++;        
        }
    }

    // Update last touch timestamp
    if (!std::static_pointer_cast<SHEPHERDReplData>(replacement_data)->shepherd_cache_flag)
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->lastTouchTick = curTick();
}

void
SHEPHERD::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    if (!std::static_pointer_cast<SHEPHERDReplData>(replacement_data)->shepherd_cache_flag)
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->lastTouchTick = curTick();
    else {
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->tickInserted = ++timeTicks;
    }
}

ReplaceableEntry*
SHEPHERD::getVictimMainCache(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    int sc_assoc = std::static_pointer_cast<SHEPHERDReplData>(candidates[0]->replacementData)->sc_associativity;
    ReplaceableEntry* victim = candidates[sc_assoc];
    for (int i = sc_assoc; i < candidates.size(); i++) {
        const auto &candidate = candidates[i];
        // Update victim entry if necessary
        if (std::static_pointer_cast<SHEPHERDReplData>(
                    candidate->replacementData)->lastTouchTick <
                std::static_pointer_cast<SHEPHERDReplData>(
                    victim->replacementData)->lastTouchTick) {
            victim = candidate;
        }
    }

    return victim;
}

ReplaceableEntry*
SHEPHERD::getVictim(const ReplacementCandidates& candidates) const
{
    std::cout << "get victim called\n";
    // victim is the block where the new data will be stored.
    // so the algorithm is like this.
    // <-->
    // if SC is not full, give a block in SC which is free
    // if SC is full, then find LRU replacement candidate from MC
    // move the FIFO candidate of SC into the MC candidate and return the free slot.

    // Visit all candidates to find victim
    int sc_assoc = std::static_pointer_cast<SHEPHERDReplData>(candidates[0]->replacementData)->sc_associativity;
    ReplaceableEntry* sc_victim = candidates[0];
    int sc_victim_index = 0;
    for (int i = 0; i < sc_assoc && i < candidates.size(); i++) {
        // Update victim entry if necessary
        const auto &candidate = candidates[i];
        if (std::static_pointer_cast<SHEPHERDReplData>(
                    candidate->replacementData)->tickInserted <
                std::static_pointer_cast<SHEPHERDReplData>(
                    sc_victim->replacementData)->tickInserted) {
            sc_victim = candidate;
            sc_victim_index = i;
        }
    }

    // now we got a fifo candidate
    int sc_set = sc_victim->getSet();
    auto sc_victim_repl_data = std::static_pointer_cast<SHEPHERDReplData>(sc_victim->replacementData);
    int nvc = sc_victim_repl_data->next_value_counter[sc_set][sc_victim_index];
    // if nvc is -1, it means we got a fresh block as victim, so we can just allocate the count matrix, increment the nvc
    // and then simply return
    if (nvc == -1) {
        sc_victim_repl_data->next_value_counter[sc_set][sc_victim_index] = 0;
        for (int i = 0; i < sc_assoc; i++) {
            if (i == sc_victim_index)
                continue;

            sc_victim_repl_data->count_value_matrix[i] = 0;
        }

        for (int i = 0; i < candidates.size(); i++) {
            std::static_pointer_cast<SHEPHERDReplData>(candidates[i]->replacementData)->count_value_matrix[sc_victim_index] = -1;
        }

        return sc_victim;
    }

    // now if nvc != -1 means you found an SC entry which was filled and needs to be emptied.
    // we can make it replace an MC line or make it replace self.
    // mc line can be chosen only if all the cells have an imminence order set
    // if not we take baseline replacement as the candidate
    bool all_imminence_order_set = true;
    for (int i = 0; i < candidates.size(); i++) {
        if (std::static_pointer_cast<SHEPHERDReplData>(candidates[i]->replacementData)->count_value_matrix[sc_victim_index] == -1) {
            all_imminence_order_set = false;
            break;
        }
    }

    // if all the imm order has been set
    // find the one with highest imminence
    if (all_imminence_order_set) {
        int max_immi = 0;
        int max_immi_index = 0;
        for (int i = 0; i < candidates.size(); i++) {
            int immi = std::static_pointer_cast<SHEPHERDReplData>(candidates[i]->replacementData)->count_value_matrix[sc_victim_index];
            if (immi > max_immi) {
                max_immi = immi;
                max_immi_index = i;
            }
        }

        // add an assert to make sure at this point max_immi_index is in a main cache only
        if (max_immi_index != sc_victim_index) {
            ReplaceableEntry *immi_victim = candidates[max_immi_index];
            immi_victim->replacementData = sc_victim->replacementData;
            std::static_pointer_cast<SHEPHERDReplData>(immi_victim->replacementData)->shepherd_cache_flag = false;
            std::static_pointer_cast<SHEPHERDReplData>(immi_victim->replacementData)->tickInserted = 0;
            std::static_pointer_cast<SHEPHERDReplData>(immi_victim->replacementData)->lastTouchTick = 0;
        }
    }
    else {
        ReplaceableEntry* mc_victim = getVictimMainCache(candidates);
        // move sc_victim to mc_victim;
        // just want to fiddle the replacement data
        mc_victim->replacementData = sc_victim->replacementData;
        std::static_pointer_cast<SHEPHERDReplData>(mc_victim->replacementData)->shepherd_cache_flag = false;
        std::static_pointer_cast<SHEPHERDReplData>(mc_victim->replacementData)->tickInserted = 0;
        std::static_pointer_cast<SHEPHERDReplData>(mc_victim->replacementData)->lastTouchTick = 0;    
    }

    // set nvc = 0
    sc_victim_repl_data->next_value_counter[sc_set][sc_victim_index] = 0;
    // set CM cells of all other SC than self as 0
    for (int i = 0; i < sc_assoc; i++) {
        if (i == sc_victim_index)
            continue;

        sc_victim_repl_data->count_value_matrix[i] = 0;
    }
    // set the CM col of this SC as -1
    for (int i = 0; i < candidates.size(); i++) {
        std::static_pointer_cast<SHEPHERDReplData>(candidates[i]->replacementData)->count_value_matrix[sc_victim_index] = -1;
    }

    return sc_victim;
}

std::shared_ptr<ReplacementData>
SHEPHERD::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new SHEPHERDReplData());
}

std::shared_ptr<ReplacementData>
SHEPHERD::instantiateEntry(bool flag, int sc_assoc, int num_set, int index_set)
{
    return std::shared_ptr<ReplacementData>(new SHEPHERDReplData(flag, sc_assoc, num_set, index_set));
}

} // namespace replacement_policy
} // namespace gem5
