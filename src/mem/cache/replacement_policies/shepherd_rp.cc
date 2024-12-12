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
    if (!std::static_pointer_cast<SHEPHERDReplData>(replacement_data)
            ->shepherd_cache_flag)
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->lastTouchTick = Tick(0);
    else
        std::static_pointer_cast<SHEPHERDReplData>(
            replacement_data)->tickInserted = 0;
}

void
SHEPHERD::touch(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // As per the implementation, nvc = -1 only when we have a cold
    // start of the cache. So, this means that the column corresponding
    // to that way has not been allocated before in the count value
    // matrix, and hence we should not set entries for that cell.
    //
    // Coming to the next point, the imminence order should only be
    // set once, when we first get a cache hit. So if the imminence
    // order for that way in count value matrix has already been
    // set, i.e., it is != -1, don't update it.
    //
    // Only in the case when the entry for that way is not set
    // yet, i.e., it is == -1, we will update it to the nvc value
    // and increment the nvc.
    auto sc_repl_data =
        std::static_pointer_cast<SHEPHERDReplData>(replacement_data);

    for (int i = 0; i < sc_repl_data->count_value_matrix.size(); i++) {
        int s_index = sc_repl_data->set_index;
        // if nvc = -1, the column is not allocated, thus skip.
        if (sc_repl_data->next_value_counter[s_index][i] == -1)
            continue;

        // if entry in count value matrix is already set, i.e., it is
        // not equal to -1, skip.
        if (sc_repl_data->count_value_matrix[i] != -1)
            continue;

        sc_repl_data->count_value_matrix[i] =
            sc_repl_data->next_value_counter[s_index][i]++;
    }

    // Update last touch timestamp
    std::static_pointer_cast<SHEPHERDReplData>(
        replacement_data)->lastTouchTick = curTick();
}

void
SHEPHERD::reset(const std::shared_ptr<ReplacementData>& replacement_data) const
{
    // Set last touch timestamp
    std::static_pointer_cast<SHEPHERDReplData>(replacement_data)
        ->lastTouchTick = curTick();

    // Set FIFO tick, only if it is a SC block
    if (std::static_pointer_cast<SHEPHERDReplData>(replacement_data)
            ->shepherd_cache_flag) {
        std::static_pointer_cast<SHEPHERDReplData>(replacement_data)
            ->tickInserted = ++timeTicks;
    }
}

ReplaceableEntry*
SHEPHERD::getVictimMainCache(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // Visit all candidates to find victim
    ReplaceableEntry* victim = candidates[0];
    for (int i = 0; i < candidates.size(); i++) {
        // Update victim entry if necessary
        if (std::static_pointer_cast<SHEPHERDReplData>(
                    candidates[i]->replacementData)->lastTouchTick <
                std::static_pointer_cast<SHEPHERDReplData>(
                    victim->replacementData)->lastTouchTick) {
            victim = candidates[i];
        }
    }

    return victim;
}

ReplaceableEntry*
SHEPHERD::getVictim(const ReplacementCandidates& candidates) const
{
    // There must be at least one replacement candidate
    assert(candidates.size() > 0);

    // [Step 1]
    // Visit all SC candidates to find victim. The victim is found
    // using FIFO order.
    int sc_assoc = std::static_pointer_cast<SHEPHERDReplData>(
        candidates[0]->replacementData)->sc_associativity;
    ReplaceableEntry* sc_victim = candidates[0];
    int sc_victim_index = 0;
    for (int i = 0; i < sc_assoc && i < candidates.size(); i++) {
        // Update victim entry if necessary
        if (std::static_pointer_cast<SHEPHERDReplData>(
                    candidates[i]->replacementData)->tickInserted <
                std::static_pointer_cast<SHEPHERDReplData>(
                    sc_victim->replacementData)->tickInserted) {
            sc_victim = candidates[i];
            sc_victim_index = i;
        }
    }

    // [Step 2]
    // Now that we got the SC victim and SC victim index, get the set
    // index of the SC victim.
    int sc_set = sc_victim->getSet();
    auto sc_victim_repl_data = std::static_pointer_cast<SHEPHERDReplData>(
        sc_victim->replacementData);

    if (sc_set != sc_victim_repl_data->set_index)
        std::cout << "<====== set index don't match ======>\n";

    int nvc = sc_victim_repl_data->
        next_value_counter[sc_set][sc_victim_index];

    // if nvc == -1, it means we got a block that has never been
    // allocated before. This should ideally happen only at a cold
    // start.
    // So now we do the following steps:
    // 1) Set nvc = 0.
    // 2) Set the count value matrix of all other ways to 0, except
    // the way of this block as it is set to -1.
    // 3) Set the Column corresponding to the current way to -1 in
    // all the candidates count value matrix.
    // 4) Since the replacement block was new, we can return it and
    // move on.
    if (nvc == -1) {
        sc_victim_repl_data->
            next_value_counter[sc_set][sc_victim_index] = 0;
        for (int i = 0; i < sc_assoc; i++) {
            if (i == sc_victim_index)
                continue;

            sc_victim_repl_data->count_value_matrix[i] = 0;
        }

        for (int i = 0; i < candidates.size(); i++) {
            std::static_pointer_cast<SHEPHERDReplData>(
                candidates[i]->replacementData)->
                    count_value_matrix[sc_victim_index] = -1;
        }

        return sc_victim;
    }

    // [Step 3]
    // If the program reached this point, it means that nvc != -1.
    // So if nvc != -1, it means we found an SC entry which was
    // in use before, and we need to empty it.
    //
    // So we can make it replace a MC cache block or self replace.
    // To choose this candidate, imminence order comes in handy.
    // But if imminence order for all ways has not been set yet,
    // then it will only help to narrow down the replacement
    // candidates. It could be one of the unset ones or self.
    // This is problematic, so we just use the baseline replacement
    // strategy to choose among the less imminent lines.
    // Please note: The imminence order of only MC entries + self SC
    // entry should be checked. Other SC entries are not replacement
    // candidates and hence should not be used.
    bool all_imminence_order_set = true;
    ReplacementCandidates narrowed_candidates;
    for (int i = sc_assoc - 1; i < candidates.size(); i++) {
        if (std::static_pointer_cast<SHEPHERDReplData>(
                candidates[i]->replacementData)->
                    count_value_matrix[sc_victim_index] == -1) {
            narrowed_candidates.push_back(candidates[i]);
            all_imminence_order_set = false;
        }
    }
    // Check if the imminence order for self SC entry is set or not.
    int sc_victim_imminence_order = std::static_pointer_cast<SHEPHERDReplData>(
            sc_victim->replacementData)->
                count_value_matrix[sc_victim_index];

    if (sc_victim_imminence_order == -1) {
        all_imminence_order_set = false;
        narrowed_candidates.push_back(sc_victim);
    }

    // If all the count value matrix entries for this SC way have
    // valid imminence order, then we can select the least imminent
    // block and do the following steps:
    // 1) If the least imminent block is self, just return self.
    // 2) If the least imminent block is not self, i.e., in MC,
    // then move the victim SC block to the least imminent MC and
    // return the victim SC block.
    ReplaceableEntry *mc_victim = nullptr;
    if (all_imminence_order_set) {
        int max_imminence = sc_victim_imminence_order;
        int max_imminence_index = sc_victim_index;
        for (int i = sc_assoc - 1; i < candidates.size(); i++) {
            int immi = std::static_pointer_cast<SHEPHERDReplData>(
                candidates[i]->replacementData)->
                    count_value_matrix[sc_victim_index];
            if (immi > max_imminence) {
                max_imminence = immi;
                max_imminence_index = i;
            }
        }

        // Check if the entry with maximum imminence order is not
        // the victim SC block itself. If it is in MC, we have to
        // move the victim SC block to the victim MC block, do that.
        if (max_imminence_index != sc_victim_index) {
            mc_victim = candidates[max_imminence_index];
        }
        // resetting of the victim SC will be handled later, before it
        // is returned. The case where the least imminent entry is
        // the victim SC block itself, we don't need to do any data
        // movement. Just need to clear SC entry and return, which is
        // handled later, so do nothing.
    }
    else {
        // This is the case where imminence order for all entries
        // has not been set. So we need to find a block in the least
        // imminent blocks using the base replacement strategy.
        mc_victim = getVictimMainCache(narrowed_candidates);
    }

    // If we found a suitable candidate in MC, do the necessary steps.
    if (mc_victim) {
        mc_victim->replacementData = sc_victim->replacementData;
        // The victim SC block moved to MC should be the next candidate for
        // replacement, so for LRU it should be least recently used. So we
        // set the lastTouchTick to be 0.
        std::static_pointer_cast<SHEPHERDReplData>(
            mc_victim->replacementData)->shepherd_cache_flag = false;
        std::static_pointer_cast<SHEPHERDReplData>(
            mc_victim->replacementData)->tickInserted = 0;
        std::static_pointer_cast<SHEPHERDReplData>(
            mc_victim->replacementData)->lastTouchTick = 0;
    }

    // Reset the nvc to 0
    sc_victim_repl_data->next_value_counter[sc_set][sc_victim_index] = 0;
    // We want to make this SC most imminent wrt to other SCs, so set
    // the count matrix value for other SCs to 0.
    for (int i = 0; i < sc_assoc; i++) {
        if (i == sc_victim_index)
            continue;

        sc_victim_repl_data->count_value_matrix[i] = 0;
    }
    // Now the reset the count matrix value for this SCs column to -1
    for (int i = 0; i < candidates.size(); i++) {
        std::static_pointer_cast<SHEPHERDReplData>(
            candidates[i]->replacementData)->
                count_value_matrix[sc_victim_index] = -1;
    }

    return sc_victim;
}

std::shared_ptr<ReplacementData>
SHEPHERD::instantiateEntry()
{
    return std::shared_ptr<ReplacementData>(new SHEPHERDReplData());
}
} // namespace replacement_policy
} // namespace gem5
