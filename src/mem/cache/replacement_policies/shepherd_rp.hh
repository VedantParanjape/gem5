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

/**
 * @file
 * Declaration of a Shepherd replacement policy.
 * TODO
 */

#ifndef __MEM_CACHE_REPLACEMENT_POLICIES_SHEPHERD_RP_HH__
#define __MEM_CACHE_REPLACEMENT_POLICIES_SHEPHERD_RP_HH__

#include <algorithm>
#include <queue>

#include "mem/cache/replacement_policies/base.hh"

namespace gem5
{

struct SHEPHERDRPParams;

namespace replacement_policy
{
class SHEPHERD : public Base
{
  protected:
    /** LRU-specific implementation of replacement data. */
    struct SHEPHERDReplData : ReplacementData
    {
        int sc_associativity;
        // set index where this data is there
        // can be used to update the next_value_counter
        int set_index;
        // MC is using LRU, so we implement LRU in that
        Tick lastTouchTick;
        // Tick on which the entry was inserted
        Tick tickInserted;
        // SC flag, to see if the line is main cache or shepherd cache
        bool shepherd_cache_flag;
        // implement sc-ptr
        // Count value matrix
        std::vector<int> count_value_matrix;
        // allocate next value counter as a set
        static std::vector<std::vector<int>> next_value_counter;
        /**
         * Default constructor. Invalidate data.
         */
        SHEPHERDReplData() : shepherd_cache_flag(false),
            sc_associativity(0), lastTouchTick(0), tickInserted(0) {}
        SHEPHERDReplData(bool flag, int sc_assoc, int num_sets,
            int index_set) : shepherd_cache_flag(flag),
              sc_associativity(sc_assoc), set_index(index_set),
              lastTouchTick(0), tickInserted(0) {
          // resize the array to shepherd cache associativity
          count_value_matrix.resize(sc_assoc);
          // set the count value matrix rows to e by default, here -1 is e
          std::fill(count_value_matrix.begin(), count_value_matrix.end(), -1);
          // resize the next value counter only once
          if (next_value_counter.size() != num_sets) {
            next_value_counter.resize(num_sets);
            for (int i = 0; i < num_sets; i++) {
              next_value_counter[i].resize(sc_assoc);
              // set the initial value of next value counter to -1
              std::fill(next_value_counter[i].begin(),
                next_value_counter[i].end(), -1);
            }
          }
        }

        void update_set_index(int s) {
        }
    };

  private:
    /**
     * A counter that tracks the number of
     * ticks since being created to avoid a tie
     */
    mutable Tick timeTicks;

  public:
    typedef SHEPHERDRPParams Params;
    SHEPHERD(const Params &p);
    ~SHEPHERD() = default;

    /**
     * Invalidate replacement data to set it as the next probable victim.
     * Sets its last touch tick as the starting tick.
     *
     * @param replacement_data Replacement data to be invalidated.
     */
    void invalidate(const std::shared_ptr<ReplacementData>& replacement_data)
                                                                    override;

    /**
     * Touch an entry to update its replacement data.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be touched.
     */
    void touch(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Reset replacement data. Used when an entry is inserted.
     * Sets its last touch tick as the current tick.
     *
     * @param replacement_data Replacement data to be reset.
     */
    void reset(const std::shared_ptr<ReplacementData>& replacement_data) const
                                                                     override;

    /**
     * Find replacement victim using LRU timestamps.
     *
     * @param candidates Replacement candidates, selected by indexing policy.
     * @return Replacement entry to be replaced.
     */
    ReplaceableEntry* getVictim(const ReplacementCandidates& candidates) const
                                                                     override;

    ReplaceableEntry* getVictimMainCache(
      const ReplacementCandidates& candidates) const;
    /**
     * Instantiate a replacement data entry.
     *
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry() override;

    /**
     * Instantiate a replacement data entry.
     *
     * @param Shepherd Cache flag
     * @param Shepherd Cache associativity
     * @return A shared pointer to the new replacement data.
     */
    std::shared_ptr<ReplacementData> instantiateEntry(
      bool flag, int sc_assoc, int num_set, int index_set);

};

} // namespace replacement_policy
} // namespace gem5

#endif // __MEM_CACHE_REPLACEMENT_POLICIES_SHEPHERD_RP_HH__
