#include "atomic.h"
#include "cpu.h"
#include "pthread.h"

static inline void ensure_locked_by_me(struct CPUState *env)
{
#if DEBUG
    if (env->atomic_memory_state->locking_cpu_id != env->id) {
        tlib_abort("Tried to release global memory lock by the cpu that does not own it!");
    }
#endif
}

static void initialize_atomic_memory_state(atomic_memory_state_t *sm)
{
    if (!sm->is_mutex_initialized) {
        pthread_mutexattr_t attributes;
        if (unlikely(pthread_mutexattr_init(&attributes))) {
            tlib_abortf("Failed to initialize phthread_muttexattr_t");
        }
// This flag is only supported on Linux
#if defined(__linux__)
        if (unlikely(pthread_mutexattr_setrobust(&attributes, PTHREAD_MUTEX_ROBUST))) {
            tlib_abortf("Failed to make the mutex robust");
        }
#endif
        if (unlikely(pthread_mutex_init(&sm->global_mutex, &attributes))) {
            tlib_abortf("Failed to initialize the pthread_mutex");
        }
        if (unlikely(pthread_mutexattr_destroy(&attributes))) {
            tlib_abortf("Failed to destroy the pthread_mutexattr");
        }

        pthread_cond_init(&sm->global_cond, NULL);
        sm->locking_cpu_id = NO_CPU_ID;
        sm->entries_count = 0;

        sm->is_mutex_initialized = 1;
    }

    if (!sm->are_reservations_valid) {
        sm->reservations_count = 0;
        for (int i = 0; i < MAX_NUMBER_OF_CPUS; i++) {
            sm->reservations[i].id = i;
            sm->reservations[i].active_flag = 0;
            sm->reservations[i].address = 0;
            sm->reservations[i].locking_cpu_id = NO_CPU_ID;
            sm->reservations[i].manual_free = 0;
            sm->reservations_by_cpu[i] = NO_RESERVATION;
        }

        sm->are_reservations_valid = 1;
    }
}

static inline address_reservation_t *find_reservation_on_address(struct CPUState *env, target_phys_addr_t address,
                                                                 int starting_position)
{
    int i;
    for (i = starting_position; i < env->atomic_memory_state->reservations_count; i++) {
        if (env->atomic_memory_state->reservations[i].address == address) {
            return &env->atomic_memory_state->reservations[i];
        }
    }
    return NULL;
}

// there can be only one reservation per cpu
static inline address_reservation_t *find_reservation_by_cpu(struct CPUState *env)
{
    int reservation_id = env->atomic_memory_state->reservations_by_cpu[env->id];
#if DEBUG
    if (reservation_id >= env->atomic_memory_state->reservations_count) {
        tlib_abort("Inconsistent reservation count detected.");
    }
#endif
    return (reservation_id == NO_RESERVATION) ? NULL : &env->atomic_memory_state->reservations[reservation_id];
}

static inline address_reservation_t *make_reservation(struct CPUState *env, target_phys_addr_t address, uint8_t manual_free)
{
    if (unlikely(env->atomic_memory_state->reservations_count == MAX_NUMBER_OF_CPUS)) {
        tlib_abort("No more address reservation slots");
    }

    address_reservation_t *reservation = &env->atomic_memory_state->reservations[env->atomic_memory_state->reservations_count];
    reservation->active_flag = 1;
    reservation->address = address;
    reservation->locking_cpu_id = env->id;
    reservation->manual_free = manual_free;

    env->atomic_memory_state->reservations_by_cpu[env->id] = env->atomic_memory_state->reservations_count;
    env->atomic_memory_state->reservations_count++;

    return reservation;
}

static inline void free_reservation(struct CPUState *env, address_reservation_t *reservation, uint8_t is_manual)
{
#if DEBUG
    if (reservation->active_flag == 0) {
        tlib_abort("Trying to free not active reservation");
    }
    if (env->atomic_memory_state->reservations_count == 0) {
        tlib_abort("Reservations count is 0, but trying to free one");
    }
#endif

    if (reservation->manual_free && !is_manual) {
        return;
    }

    env->atomic_memory_state->reservations_by_cpu[reservation->locking_cpu_id] = NO_RESERVATION;
    if (reservation->id != env->atomic_memory_state->reservations_count - 1) {
        // if this is not the last reservation, i must copy the last one in this empty place
        reservation->locking_cpu_id =
            env->atomic_memory_state->reservations[env->atomic_memory_state->reservations_count - 1].locking_cpu_id;
        reservation->address = env->atomic_memory_state->reservations[env->atomic_memory_state->reservations_count - 1].address;
        // active flag does not have to be copied as it's always 1

        // and update mapping
        env->atomic_memory_state->reservations_by_cpu[reservation->locking_cpu_id] = reservation->id;
    }

    env->atomic_memory_state->reservations[env->atomic_memory_state->reservations_count - 1].active_flag = 0;
    env->atomic_memory_state->reservations_count--;
}


void register_in_atomic_memory_state(atomic_memory_state_t *sm, int id)
{
    if (id >= MAX_NUMBER_OF_CPUS) {
        tlib_abortf("Cpu id: %d exceeds the number of supported cores: %d", id, MAX_NUMBER_OF_CPUS);
    }

    initialize_atomic_memory_state(sm);
}

void acquire_global_memory_lock(struct CPUState *env)
{
    if (env->atomic_memory_state == NULL) {
        // no atomic_memory_state so no need for synchronization
        return;
    }

    pthread_mutex_lock(&env->atomic_memory_state->global_mutex);
    if (env->atomic_memory_state->locking_cpu_id != env->id) {
        while (env->atomic_memory_state->locking_cpu_id != NO_CPU_ID) {
            pthread_cond_wait(&env->atomic_memory_state->global_cond, &env->atomic_memory_state->global_mutex);
        }
        env->atomic_memory_state->locking_cpu_id = env->id;
    }
    env->atomic_memory_state->entries_count++;
    pthread_mutex_unlock(&env->atomic_memory_state->global_mutex);
}

void release_global_memory_lock(struct CPUState *env)
{
    if (env->atomic_memory_state == NULL) {
        // no atomic_memory_state so no need for synchronization
        return;
    }

    pthread_mutex_lock(&env->atomic_memory_state->global_mutex);
    ensure_locked_by_me(env);
    env->atomic_memory_state->entries_count--;
    if (env->atomic_memory_state->entries_count == 0) {
        env->atomic_memory_state->locking_cpu_id = NO_CPU_ID;
        pthread_cond_signal(&env->atomic_memory_state->global_cond);
    }
    pthread_mutex_unlock(&env->atomic_memory_state->global_mutex);
}

void clear_global_memory_lock(struct CPUState *env)
{
    pthread_mutex_lock(&env->atomic_memory_state->global_mutex);
    ensure_locked_by_me(env);
    env->atomic_memory_state->locking_cpu_id = NO_CPU_ID;
    env->atomic_memory_state->entries_count = 0;
    pthread_cond_signal(&env->atomic_memory_state->global_cond);
    pthread_mutex_unlock(&env->atomic_memory_state->global_mutex);
}

// ! this function should be called when holding the mutex !
// If manual_free is true then the performed reservation will only be able to be cancelled explicitly,
// by calling `cancel_reservation` or by performing a different reservation on a CPU that already had
// had a reserved address.
void reserve_address(struct CPUState *env, target_phys_addr_t address, uint8_t manual_free)
{
    ensure_locked_by_me(env);

    address_reservation_t *reservation = find_reservation_by_cpu(env);
    if (reservation != NULL) {
        if (reservation->address == address) {
            return;
        }
        // cancel the previous reservation and set a new one
        free_reservation(env, reservation, 1);
    }
    make_reservation(env, address, manual_free);
}

uint32_t check_address_reservation(struct CPUState *env, target_phys_addr_t address)
{
    ensure_locked_by_me(env);
    address_reservation_t *reservation = find_reservation_by_cpu(env);
    return (reservation == NULL || reservation->address != address);
}

void register_address_access(struct CPUState *env, target_phys_addr_t address)
{
    if (env->atomic_memory_state == NULL) {
        // no atomic_memory_state so no registration needed
        return;
    }

    ensure_locked_by_me(env);

    address_reservation_t *reservation = find_reservation_on_address(env, address, 0);
    while (reservation != NULL) {
        if (reservation->locking_cpu_id != env->id) {
            free_reservation(env, reservation, 0);
        }
        reservation = find_reservation_on_address(env, address, reservation->id + 1);
    }
}

void cancel_reservation(struct CPUState *env)
{
    ensure_locked_by_me(env);

    address_reservation_t *reservation = find_reservation_by_cpu(env);
    if (reservation != NULL) {
        free_reservation(env, reservation, 1);
    }
}
