# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Sub-module containing utilities for transforming strings and regular expressions."""

from __future__ import annotations

import ast
import importlib
import inspect
import re
from typing import Any, Callable, Sequence

"""
String formatting.
"""


def to_camel_case(snake_str: str, to: str = "cC") -> str:
    """Converts a string from snake case to camel case.

    Args:
        snake_str: A string in snake case (i.e. with '_')
        to: Convention to convert string to. Defaults to "cC".

    Raises:
        ValueError: Invalid input argument `to`, i.e. not "cC" or "CC".

    Returns:
        A string in camel-case format.
    """
    # check input is correct
    if to not in ["cC", "CC"]:
        msg = "to_camel_case(): Choose a valid `to` argument (CC or cC)"
        raise ValueError(msg)
    # convert string to lower case and split
    components = snake_str.lower().split("_")
    if to == "cC":
        # We capitalize the first letter of each component except the first one
        # with the 'title' method and join them together.
        return components[0] + "".join(x.title() for x in components[1:])
    else:
        # Capitalize first letter in all the components
        return "".join(x.title() for x in components)


def to_snake_case(camel_str: str) -> str:
    """Converts a string from camel case to snake case.

    Args:
        camel_str: A string in camel case.

    Returns:
        A string in snake case (i.e. with '_')
    """
    camel_str = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", camel_str)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", camel_str).lower()


"""
String <-> Callable operations.
"""


def is_lambda_expression(name: str) -> bool:
    """Checks if the input string is a lambda expression.

    Args:
        name: The input string.

    Returns:
        Whether the input string is a lambda expression.
    """
    try:
        ast.parse(name)
        return isinstance(ast.parse(name).body[0], ast.Expr) and isinstance(ast.parse(name).body[0].value, ast.Lambda)
    except SyntaxError:
        return False


def callable_to_string(value: Callable) -> str:
    """Converts a callable object to a string.

    Args:
        value: A callable object.

    Raises:
        ValueError: When the input argument is not a callable object.

    Returns:
        A string representation of the callable object.
    """
    # check if callable
    if not callable(value):
        raise ValueError(f"The input argument is not callable: {value}.")
    # check if lambda function
    if value.__name__ == "<lambda>":
        return f"lambda {inspect.getsourcelines(value)[0][0].strip().split('lambda')[1].strip().split(',')[0]}"
    else:
        # get the module and function name
        module_name = value.__module__
        function_name = value.__name__
        # return the string
        return f"{module_name}:{function_name}"


def string_to_callable(name: str) -> Callable:
    """Resolves the module and function names to return the function.

    Args:
        name: The function name. The format should be 'module:attribute_name' or a
            lambda expression of format: 'lambda x: x'.

    Raises:
        ValueError: When the resolved attribute is not a function.
        ValueError: When the module cannot be found.

    Returns:
        Callable: The function loaded from the module.
    """
    try:
        if is_lambda_expression(name):
            callable_object = eval(name)
        else:
            mod_name, attr_name = name.split(":")
            mod = importlib.import_module(mod_name)
            callable_object = getattr(mod, attr_name)
        # check if attribute is callable
        if callable(callable_object):
            return callable_object
        else:
            raise AttributeError(f"The imported object is not callable: '{name}'")
    except (ValueError, ModuleNotFoundError) as e:
        msg = (
            f"Could not resolve the input string '{name}' into callable object."
            " The format of input should be 'module:attribute_name'.\n"
            f"Received the error:\n {e}."
        )
        raise ValueError(msg)


"""
Regex operations.
"""


def resolve_matching_names(keys: str | Sequence[str], list_of_strings: Sequence[str]) -> tuple[list[int], list[str]]:
    """Match a list of query regular expressions against a list of strings and return the matched indices and names.

    When a list of query regular expressions is provided, the function checks each target string against each
    query regular expression and returns the indices of the matched strings and the matched strings.
    This means that the ordering is dictated by the order of the target strings and not the order of the query
    regular expressions.

    For example, if the list of strings is ['a', 'b', 'c', 'd', 'e'] and the regular expressions are ['a|c', 'b'],
    then the function will return the indices of the matched strings and the matched strings, i.e.
    ([0, 1, 2], ['a', 'b', 'c']).

    Note:
        The function does not sort the indices. It returns the indices in the order they are found.

    Args:
        keys: A regular expression or a list of regular expressions to match the strings in the list.
        list_of_strings: A list of strings to match.

    Returns:
        A tuple of lists containing the matched indices and names.

    Raises:
        ValueError: When multiple matches are found for a string in the list.
        ValueError: When not all regular expressions are matched.
    """
    # resolve name keys
    if isinstance(keys, str):
        keys = [keys]
    # find matching patterns
    index_list = []
    names_list = []
    # book-keeping to check that we always have a one-to-one mapping
    # i.e. each target string should match only one regular expression
    target_strings_match_found = [None for _ in range(len(list_of_strings))]
    keys_match_found = [[] for _ in range(len(keys))]
    # loop over all target strings
    for target_index, potential_match_string in enumerate(list_of_strings):
        for key_index, re_key in enumerate(keys):
            if re.fullmatch(re_key, potential_match_string):
                # check if match already found
                if target_strings_match_found[target_index]:
                    raise ValueError(
                        f"Multiple matches for '{potential_match_string}':"
                        f" '{target_strings_match_found[target_index]}' and '{re_key}'!"
                    )
                # add to list
                target_strings_match_found[target_index] = re_key
                index_list.append(target_index)
                names_list.append(potential_match_string)
                # add for regex key
                keys_match_found[key_index].append(potential_match_string)
    # check that all regular expressions are matched
    if not all(keys_match_found):
        # make this print nicely aligned for debugging
        msg = "\n"
        for key, value in zip(keys, keys_match_found):
            msg += f"\t{key}: {value}\n"
        msg += f"Available strings: {list_of_strings}\n"
        # raise error
        raise ValueError(
            f"Not all regular expressions are matched! Please check that the regular expressions are correct: {msg}"
        )
    # return
    return index_list, names_list


def resolve_matching_names_values(
    data: dict[str, Any], list_of_strings: Sequence[str]
) -> tuple[list[int], list[str], list[Any]]:
    """Match a list of regular expressions in a dictionary against a list of strings and return
    the matched indices, names, and values.

    For example, if the dictionary is {'a|b|c': 1, 'd|e': 2} and the list of strings is ['a', 'b', 'c', 'd', 'e'],
    then the function will return the indices of the matched strings, the matched strings, and the values, i.e.
    ([0, 1, 2, 3, 4], ['a', 'b', 'c', 'd', 'e'], [1, 1, 1, 2, 2]).

    Note:
        The function does not sort the indices. It returns the indices in the order they are found.

    Args:
        data: A dictionary of regular expressions and values to match the strings in the list.
        list_of_strings: A list of strings to match.

    Returns:
        A tuple of lists containing the matched indices, names, and values.

    Raises:
        TypeError: When the input argument `data` is not a dictionary.
        ValueError: When multiple matches are found for a string in the dictionary.
        ValueError: When not all regular expressions in the data keys are matched.
    """
    # check valid input
    if not isinstance(data, dict):
        raise TypeError(f"Input argument `data` should be a dictionary. Received: {data}")
    # find matching patterns
    index_list = []
    names_list = []
    values_list = []
    # book-keeping to check that we always have a one-to-one mapping
    # i.e. each target string should match only one regular expression
    target_strings_match_found = [None for _ in range(len(list_of_strings))]
    keys_match_found = [[] for _ in range(len(data))]
    # loop over all target strings
    for target_index, potential_match_string in enumerate(list_of_strings):
        for key_index, (re_key, value) in enumerate(data.items()):
            if re.fullmatch(re_key, potential_match_string):
                # check if match already found
                if target_strings_match_found[target_index]:
                    raise ValueError(
                        f"Multiple matches for '{potential_match_string}':"
                        f" '{target_strings_match_found[target_index]}' and '{re_key}'!"
                    )
                # add to list
                target_strings_match_found[target_index] = re_key
                index_list.append(target_index)
                names_list.append(potential_match_string)
                values_list.append(value)
                # add for regex key
                keys_match_found[key_index].append(potential_match_string)
    # check that all regular expressions are matched
    if not all(keys_match_found):
        # make this print nicely aligned for debugging
        msg = "\n"
        for key, value in zip(data.keys(), keys_match_found):
            msg += f"\t{key}: {value}\n"
        msg += f"Available strings: {list_of_strings}\n"
        # raise error
        raise ValueError(
            f"Not all regular expressions are matched! Please check that the regular expressions are correct: {msg}"
        )
    # return
    return index_list, names_list, values_list
