#ifndef DMAPF_INCLUDE_COMMON_H_
#define DMAPF_INCLUDE_COMMON_H_

#include <memory>
#include <string>
#include <vector>

#include <clingo.hh>

// Get the program literal corresponding to the external atom
void get_literal(const clingo_control_t* ctl, const clingo_symbol_t sym, clingo_literal_t* atom)
{
  const clingo_symbolic_atoms_t* atoms;
  clingo_symbolic_atom_iterator_t atom_it;

  clingo_control_symbolic_atoms(ctl, &atoms);
  clingo_symbolic_atoms_find(atoms, sym, &atom_it);
  clingo_symbolic_atoms_literal(atoms, atom_it, atom);
}

bool get_model_string(const clingo_model_t* model, std::string& model_string, const std::string& spacer = ".\n")
{
  bool ret = true;
  clingo_symbol_t *atoms = NULL;
  size_t atoms_n;
  clingo_symbol_t const *it, *ie;
  char *str = NULL;
  size_t str_n = 0;

  // Determine the number of (shown) symbols in the model
  if (!clingo_model_symbols_size(model, clingo_show_type_shown, &atoms_n)) goto error;

  // Allocate required memory to hold all the symbols
  if (!(atoms = (clingo_symbol_t*)malloc(sizeof(*atoms) * atoms_n)))
  {
    clingo_set_error(clingo_error_bad_alloc, "could not allocate memory for atoms");
    goto error;
  }

  // Retrieve the symbols in the model
  if (!clingo_model_symbols(model, clingo_show_type_shown, atoms, atoms_n)) goto error;

  for (it = atoms, ie = atoms + atoms_n; it != ie; ++it)
  {
    size_t n;
    char *str_new;

    // Determine size of the string representation of the next symbol in the model
    if (!clingo_symbol_to_string_size(*it, &n)) goto error;
    if (str_n < n)
    {
      // Allocate required memory to hold the symbol's string
      if (!(str_new = (char*)realloc(str, sizeof(*str) * n)))
      {
        clingo_set_error(clingo_error_bad_alloc, "could not allocate memory for symbol's string");
        goto error;
      }
      str = str_new;
      str_n = n;
    }

    // Retrieve the symbol's string
    if (!clingo_symbol_to_string(*it, str, n)) goto error;

    model_string += std::string(str) + spacer;
  }
  goto out;

  error:
  ret = false;

  out:
  if (atoms)
    free(atoms);
  if (str)
    free(str);
  return ret;
}

// Solve and get a model string
bool solve(clingo_control_t* ctl, clingo_solve_result_bitset_t* result, std::string& model_string, const std::string& spacer = "")
{
  bool ret = true;
  const clingo_model_t* last_valid_model = NULL;

  // Get a solve handle
  clingo_solve_handle_t* handle;
  if (!clingo_control_solve(ctl, clingo_solve_mode_yield, NULL, 0, NULL, NULL, &handle)) goto error;

  while (true)
  {
    const clingo_model_t* model = NULL;
    if (!clingo_solve_handle_resume(handle)) goto error;
    if (!clingo_solve_handle_model(handle, &model)) goto error;

    if (!model)   // Stop if there is no more model
      break;

    last_valid_model = model;   // Save the last valid model
  }

  // Close the solve handle
  if (!clingo_solve_handle_get(handle, result)) goto error;

  // Get the model string
  if (last_valid_model && !get_model_string(last_valid_model, model_string, spacer)) goto error;

  goto out;

  error:
  ret = false;

  out:
  // Free the solve handle
  return clingo_solve_handle_close(handle) && ret;
}

// Solve within a time limit and get a model string
bool solve(clingo_control_t* ctl, clingo_solve_result_bitset_t* result, const double timeout, std::string& model_string, const std::string& spacer = "")
{
  bool ret = true;
  clock_t time_start = clock();
  const clingo_model_t* last_valid_model = NULL;

  // Get a solve handle
  clingo_solve_handle_t* handle;
  if (!clingo_control_solve(ctl, clingo_solve_mode_async | clingo_solve_mode_yield, NULL, 0, NULL, NULL, &handle)) goto error;

  while (true)
  {
    const clingo_model_t* model = NULL;
    if (!clingo_solve_handle_resume(handle)) goto error;

    // Wait for the remaining time
    double remaining_time = timeout - (double)(clock() - time_start) / CLOCKS_PER_SEC;
    bool finished = false;
    if (remaining_time > 0.0)
    {
      clingo_solve_handle_wait(handle, remaining_time, &finished);
    }
    if (!finished)
    {
      clingo_solve_handle_cancel(handle);
      goto error;
    }

    if (!clingo_solve_handle_model(handle, &model)) goto error;

    if (!model)   // Stop if there is no more model
      break;

    last_valid_model = model;   // Save the last valid model
  }

  // Close the solve handle
  if (!clingo_solve_handle_get(handle, result)) goto error;

  // Get the model string
  if (last_valid_model && !get_model_string(last_valid_model, model_string, spacer)) goto error;

  goto out;

  error:
  ret = false;

  out:
  // Free the solve handle
  return clingo_solve_handle_close(handle) && ret;
}

// Only solve
bool solve(clingo_control_t* ctl, clingo_solve_result_bitset_t* result)
{
  bool ret = true;

  // Get a solve handle
  clingo_solve_handle_t* handle;
  if (!clingo_control_solve(ctl, clingo_solve_mode_yield, NULL, 0, NULL, NULL, &handle)) goto error;

  while (true)
  {
    const clingo_model_t* model = NULL;
    if (!clingo_solve_handle_resume(handle)) goto error;
    if (!clingo_solve_handle_model(handle, &model)) goto error;

    if (!model)   // Stop if there is no more model
      break;
  }

  // Close the solve handle
  if (!clingo_solve_handle_get(handle, result)) goto error;

  goto out;

  error:
  ret = false;

  out:
  // Free the solve handle
  return clingo_solve_handle_close(handle) && ret;
}

// Only solve within a time limit
bool solve(clingo_control_t* ctl, clingo_solve_result_bitset_t* result, const double timeout)
{
  bool ret = true;
  clock_t time_start = clock();

  // Get a solve handle
  clingo_solve_handle_t* handle;
  if (!clingo_control_solve(ctl, clingo_solve_mode_async | clingo_solve_mode_yield, NULL, 0, NULL, NULL, &handle)) goto error;

  while (true)
  {
    const clingo_model_t* model = NULL;
    if (!clingo_solve_handle_resume(handle)) goto error;

    // Wait for the remaining time
    double remaining_time = timeout - (double)(clock() - time_start) / CLOCKS_PER_SEC;
    bool finished = false;
    if (remaining_time > 0.0)
    {
      clingo_solve_handle_wait(handle, remaining_time, &finished);
    }
    if (!finished)
    {
      clingo_solve_handle_cancel(handle);
      goto error;
    }

    if (!clingo_solve_handle_model(handle, &model)) goto error;

    if (!model)   // Stop if there is no more model
      break;
  }

  // Close the solve handle
  if (!clingo_solve_handle_get(handle, result)) goto error;

  goto out;

  error:
  ret = false;

  out:
  // Free the solve handle
  return clingo_solve_handle_close(handle) && ret;
}

#endif
