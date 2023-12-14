# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at

#   http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License. 

"""Module for the TextFormatting substitution."""

from typing import Text
from typing import Iterable

from launch.launch_context import LaunchContext
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions
from launch.some_substitutions_type import SomeSubstitutionsType


class TextFormatting(Substitution):
    """Substitution that wraps a single string text."""

    def __init__(self, text: Text, substitutions: Iterable[SomeSubstitutionsType]) -> None:
        """Create a TextSubstitution."""
        super().__init__()

        if not isinstance(text, Text):
            raise TypeError(
                "TextSubstitution expected Text object got '{}' instead.".format(type(text))
            )

        self.__text = text
        self.__substitutions = normalize_to_list_of_substitutions(substitutions)

    @property
    def substitutions(self) -> Iterable[Substitution]:
        """Getter for variable_name."""
        return self.__substitutions

    @property
    def text(self) -> Text:
        """Getter for text."""
        return self.__text

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return "{}%({})".format(self.text, ', '.join([s.describe() for s in self.substitutions]))

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by retrieving the local variable."""
        performed_substitutions = [sub.perform(context) for sub in self.__substitutions]
        return self.text.format(*performed_substitutions)